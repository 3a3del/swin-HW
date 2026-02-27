// =============================================================================
// conv_controller.sv  (fixed — missing comma corrected, logic unchanged)
// Main FSM controller for the MMU-based convolution engine.
//
// ── Convolution parameters ────────────────────────────────────────────────
//   Image  : 224×224×3 (CHW layout in memory)
//   Kernel : 4×4×3 × 96 kernels
//   Stride : 4  →  Output: 56×56×96
//   MMU    : 12 PEs (4 rows × 3 channels), 7 output columns per chunk
//
// ── Processing order ─────────────────────────────────────────────────────
//   for kernel_idx   0..95
//     for row_group  0..55
//       for chunk    0..7
//         LOAD_W  (26 cyc)  → SWAP_W  (1) →
//         LOAD_IMG(168 cyc) → SWAP_IMG(1) →
//         COMPUTE (1) → WAIT_OUT(1, capture) →
//         WRITEBACK(7) → NEXT(1)
//
//   Weight load (13 words) only when kernel_idx changes.
//
// ── Memory timing ─────────────────────────────────────────────────────────
//   1-cycle read latency: rd_data valid on cycle AFTER rd_en.
//   Each word = 2 cycles: even load_cyc → send addr+rd_en;
//                         odd  load_cyc → capture rd_data → write buffer.
// =============================================================================

module conv_controller (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,

    // ── Weight memory interface ────────────────────────────────────────────
    output logic [31:0] wmem_addr,
    output logic        wmem_rd_en,
    input  logic [31:0] wmem_rd_data,

    // ── Image (FIB) memory interface ──────────────────────────────────────
    output logic [31:0] imem_addr,
    output logic        imem_rd_en,
    input  logic [31:0] imem_rd_data,

    // ── Output memory interface ────────────────────────────────────────────
    output logic [31:0] omem_addr,
    output logic        omem_wr_en,

    // ── Weight buffer control ──────────────────────────────────────────────
    output logic        wbuf_load_en,
    output logic [3:0]  wbuf_load_pe_idx,
    output logic [31:0] wbuf_load_data,
    output logic        wbuf_bias_load_en,
    output logic [31:0] wbuf_bias_load_data,
    output logic        wbuf_swap,

    // ── Input (line) buffer control ───────────────────────────────────────
    output logic        ibuf_load_en,
    output logic [3:0]  ibuf_load_pe_idx,
    output logic [2:0]  ibuf_load_win_idx,
    output logic [31:0] ibuf_load_data,
    output logic        ibuf_swap,

    // ── MMU control ───────────────────────────────────────────────────────
    output logic        mmu_valid_in,
    output logic        mmu_capture_en,

    // ── Output buffer readout index ───────────────────────────────────────
    output logic [2:0]  obuf_rd_idx
);

// =============================================================================
// Parameters
// =============================================================================
localparam int N_KERNELS    = 96;
localparam int N_ROW_GROUPS = 56;   // 224/4
localparam int N_CHUNKS     = 8;    // 224/28
localparam int WLOAD_WORDS  = 13;   // 12 weights + 1 bias
localparam int ILOAD_WORDS  = 84;   // 12 PEs × 7 words
localparam int OUT_WORDS    = 7;

localparam int IMG_W_WORDS  = 56;
localparam int IMG_CH_WORDS = 56 * 224;
localparam int CHUNK_WORDS  = 7;
localparam int OUT_ROW_WORDS= 56;
localparam int OUT_K_WORDS  = 56 * 56;

localparam int WLOAD_CYCS   = 2 * WLOAD_WORDS; // 26
localparam int ILOAD_CYCS   = 2 * ILOAD_WORDS; // 168

// =============================================================================
// State encoding
// =============================================================================
typedef enum logic [3:0] {
    S_IDLE      = 4'd0,
    S_LOAD_W    = 4'd1,
    S_SWAP_W    = 4'd2,
    S_LOAD_IMG  = 4'd3,
    S_SWAP_IMG  = 4'd4,
    S_COMPUTE   = 4'd5,
    S_WAIT_OUT  = 4'd6,
    S_WRITEBACK = 4'd7,
    S_NEXT      = 4'd8,
    S_DONE      = 4'd9
} state_t;

state_t state, next_state;

// =============================================================================
// Counters
// =============================================================================
logic [6:0] kernel_idx;
logic [5:0] row_group_idx;
logic [2:0] chunk_idx;
logic [7:0] load_cyc;
logic [2:0] wb_cnt;

logic [6:0] load_cnt;
logic       is_data_ph;
assign load_cnt   = load_cyc[7:1];
assign is_data_ph = load_cyc[0];

// =============================================================================
// Address generation
// =============================================================================

// Weight memory: word = kernel*13 + load_cnt
always_comb begin
    wmem_addr = 32'(kernel_idx) * WLOAD_WORDS + 32'(load_cnt);
end

// Image memory: decompose load_cnt → pe, win, ch, row
logic [3:0] img_pe;
logic [2:0] img_win;
logic [1:0] img_ch;
logic [7:0] img_row;

always_comb begin
    img_pe    = 4'(load_cnt / 7);
    img_win   = 3'(load_cnt % 7);
    img_ch    = img_pe[3:2];
    img_row   = {2'b0, row_group_idx} * 4 + {6'b0, img_pe[1:0]};
    imem_addr = 32'(img_ch)  * IMG_CH_WORDS
              + 32'(img_row) * IMG_W_WORDS
              + 32'(chunk_idx) * CHUNK_WORDS
              + 32'(img_win);
end

// Output memory: word = kernel*3136 + row_group*56 + chunk*7 + wb_cnt
always_comb begin
    omem_addr = 32'(kernel_idx)    * OUT_K_WORDS
              + 32'(row_group_idx) * OUT_ROW_WORDS
              + 32'(chunk_idx)     * OUT_WORDS
              + 32'(wb_cnt);
end

// =============================================================================
// State register
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;
end

// =============================================================================
// Combinatorial flags
// =============================================================================
logic is_last_chunk, is_last_row_group, is_last_kernel;
assign is_last_chunk     = (chunk_idx     == N_CHUNKS     - 1);
assign is_last_row_group = (row_group_idx == N_ROW_GROUPS - 1);
assign is_last_kernel    = (kernel_idx    == N_KERNELS    - 1);

// =============================================================================
// Next-state logic
// =============================================================================
always_comb begin
    next_state = state;
    case (state)
        S_IDLE:      if (start)                               next_state = S_LOAD_W;
        S_LOAD_W:    if (load_cyc == WLOAD_CYCS - 1)         next_state = S_SWAP_W;
        S_SWAP_W:                                             next_state = S_LOAD_IMG;
        S_LOAD_IMG:  if (load_cyc == ILOAD_CYCS - 1)         next_state = S_SWAP_IMG;
        S_SWAP_IMG:                                           next_state = S_COMPUTE;
        S_COMPUTE:                                            next_state = S_WAIT_OUT;
        S_WAIT_OUT:                                           next_state = S_WRITEBACK;
        S_WRITEBACK: if (wb_cnt == OUT_WORDS - 1)            next_state = S_NEXT;
        S_NEXT: begin
            if (is_last_chunk && is_last_row_group && is_last_kernel)
                next_state = S_DONE;
            else if (is_last_chunk && is_last_row_group)
                next_state = S_LOAD_W;
            else
                next_state = S_LOAD_IMG;
        end
        S_DONE:   next_state = S_DONE;
        default:  next_state = S_IDLE;
    endcase
end

// =============================================================================
// Counter updates
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        kernel_idx    <= '0;
        row_group_idx <= '0;
        chunk_idx     <= '0;
        load_cyc      <= '0;
        wb_cnt        <= '0;
    end else begin
        case (state)
            S_IDLE, S_SWAP_W, S_SWAP_IMG:
                load_cyc <= '0;

            S_LOAD_W:
                load_cyc <= (load_cyc < WLOAD_CYCS-1) ? load_cyc+1 : '0;

            S_LOAD_IMG:
                load_cyc <= (load_cyc < ILOAD_CYCS-1) ? load_cyc+1 : '0;

            S_WRITEBACK:
                wb_cnt <= (wb_cnt < OUT_WORDS-1) ? wb_cnt+1 : '0;

            S_NEXT: begin
                if (!is_last_chunk) begin
                    chunk_idx <= chunk_idx + 1;
                end else begin
                    chunk_idx <= '0;
                    if (!is_last_row_group) begin
                        row_group_idx <= row_group_idx + 1;
                    end else begin
                        row_group_idx <= '0;
                        if (!is_last_kernel)
                            kernel_idx <= kernel_idx + 1;
                    end
                end
            end

            default: ;
        endcase
    end
end

// =============================================================================
// Output / control signal generation
// =============================================================================
assign done = (state == S_DONE);

// Weight memory
assign wmem_rd_en          = (state == S_LOAD_W) && !is_data_ph;

// Weight buffer
assign wbuf_load_en        = (state == S_LOAD_W) &&  is_data_ph && (load_cnt < 12);
assign wbuf_load_pe_idx    = 4'(load_cnt);
assign wbuf_load_data      = wmem_rd_data;

assign wbuf_bias_load_en   = (state == S_LOAD_W) &&  is_data_ph && (load_cnt == 12);
assign wbuf_bias_load_data = wmem_rd_data;

assign wbuf_swap           = (state == S_SWAP_W);

// Image memory
assign imem_rd_en          = (state == S_LOAD_IMG) && !is_data_ph;

// Input buffer
assign ibuf_load_en        = (state == S_LOAD_IMG) && is_data_ph;
assign ibuf_load_pe_idx    = img_pe;
assign ibuf_load_win_idx   = img_win;
assign ibuf_load_data      = imem_rd_data;
assign ibuf_swap           = (state == S_SWAP_IMG);

// MMU
assign mmu_valid_in        = (state == S_COMPUTE) || (state == S_WAIT_OUT);
assign mmu_capture_en      = (state == S_WAIT_OUT);

// Output buffer
assign obuf_rd_idx         = wb_cnt;

// Output memory
assign omem_wr_en          = (state == S_WRITEBACK);

endmodule
