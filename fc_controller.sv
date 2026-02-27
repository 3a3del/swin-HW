// =============================================================================
// fc_controller.sv  (fixed — missing comma corrected, logic unchanged)
// FSM controller for the two-layer MLP engine.
//
// ── MLP parameters ────────────────────────────────────────────────────────
//   Layer-1 : X(3136×96)  × W1(96×384)  → Y(3136×384)
//   Layer-2 : Y(3136×384) × W2(384×96)  → Z(3136×96)
//   Row groups: 3136/7 = 448
//
// ── MMU mapping ───────────────────────────────────────────────────────────
//   48 mul/cycle (12 PEs × 4 taps)
//   L1: N_ACC = 96/48  = 2 sub-cycles  → op_code=1, stage=0
//   L2: N_ACC = 384/48 = 8 sub-cycles  → op_code=1, stage=2
//
//   valid_in held for N_ACC+1 cycles; capture fires on cycle N_ACC.
//
// ── Memory layouts (word-addressed, 4B/word) ─────────────────────────────
//   W1  (col-major): addr = l1_col*24 + k_word      col∈0..383
//   W2  (col-major): addr = l2_col*96 + k_word      col∈0..95
//                    (DSU adds W2_BASE=9216 before presenting to weight_memory)
//   X   (row-major): addr = (row_grp*7+sub_row)*24 + k_word
//   Out (Z)        : addr = l2_col*3136 + row_grp*7 + wb_cnt
// =============================================================================

module fc_controller (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,

    // ── W1 memory interface (via DSU → weight_memory region 0) ───────────
    output logic [31:0] w1mem_addr,
    output logic        w1mem_rd_en,
    input  logic [31:0] w1mem_rd_data,

    // ── W2 memory interface (via DSU → weight_memory region 1) ───────────
    output logic [31:0] w2mem_addr,
    output logic        w2mem_rd_en,
    input  logic [31:0] w2mem_rd_data,

    // ── X (FIB) memory interface ──────────────────────────────────────────
    output logic [31:0] xmem_addr,
    output logic        xmem_rd_en,
    input  logic [31:0] xmem_rd_data,

    // ── Output memory interface ────────────────────────────────────────────
    output logic [31:0] omem_addr,
    output logic        omem_wr_en,

    // ── Unified weight buffer control ─────────────────────────────────────
    output logic        wbuf_load_en,
    output logic [6:0]  wbuf_load_k_word,
    output logic [31:0] wbuf_load_data,
    output logic        wbuf_swap,

    // ── Unified input buffer control (X load) ─────────────────────────────
    output logic        ibuf_load_en,
    output logic [2:0]  ibuf_load_row,
    output logic [6:0]  ibuf_load_k_word,
    output logic [31:0] ibuf_load_data,
    output logic        ibuf_swap,

    // ── Unified input buffer: L1 Y capture (shadow bank during L1 compute)─
    output logic        ibuf_l1_capture_en,
    output logic [8:0]  ibuf_l1_col_wr,

    // ── MMU control ───────────────────────────────────────────────────────
    output logic        mmu_valid_in,
    output logic [2:0]  mmu_op_code,
    output logic [1:0]  mmu_stage,
    output logic [2:0]  mmu_sub_cycle,

    // ── L2 output capture ─────────────────────────────────────────────────
    output logic        l2_capture_en,
    output logic [2:0]  obuf_rd_idx
);

// =============================================================================
// Parameters
// =============================================================================
localparam int N_ROW_GRPS    = 448;
localparam int N_L1_COLS     = 384;
localparam int N_L2_COLS     = 96;
localparam int N_ACC_L1      = 2;
localparam int N_ACC_L2      = 8;

localparam int W1_LOAD_WORDS = 24;
localparam int W2_LOAD_WORDS = 96;
localparam int X_LOAD_WORDS  = 168;

localparam int W1_LOAD_CYCS  = W1_LOAD_WORDS * 2;   // 48
localparam int W2_LOAD_CYCS  = W2_LOAD_WORDS * 2;   // 192
localparam int X_LOAD_CYCS   = X_LOAD_WORDS  * 2;   // 336

localparam int OUT_WORDS     = 7;

// =============================================================================
// State encoding
// =============================================================================
typedef enum logic [4:0] {
    S_IDLE         = 5'd0,
    S_L1_LOAD_X    = 5'd1,
    S_L1_SWAP_X    = 5'd2,
    S_L1_LOAD_W    = 5'd3,
    S_L1_SWAP_W    = 5'd4,
    S_L1_COMPUTE   = 5'd5,
    S_L1_NEXT_COL  = 5'd6,
    S_L2_LOAD_W    = 5'd7,
    S_L2_SWAP_W    = 5'd8,
    S_L2_COMPUTE   = 5'd9,
    S_L2_WRITEBACK = 5'd10,
    S_L2_NEXT_COL  = 5'd11,
    S_NEXT_ROW_GRP = 5'd12,
    S_DONE         = 5'd13
} state_t;

state_t state, next_state;

// =============================================================================
// Counters
// =============================================================================
logic [8:0] row_grp_idx;
logic [8:0] l1_col_idx;
logic [6:0] l2_col_idx;
logic [8:0] load_cyc;
logic [3:0] compute_cnt;
logic [2:0] wb_cnt;

logic       is_data_ph;
logic [7:0] load_cnt;
assign is_data_ph = load_cyc[0];
assign load_cnt   = load_cyc[8:1];

// =============================================================================
// Last-element flags
// =============================================================================
logic is_last_row_grp, is_last_l1_col, is_last_l2_col, is_last_wb;
assign is_last_row_grp = (row_grp_idx == N_ROW_GRPS - 1);
assign is_last_l1_col  = (l1_col_idx  == N_L1_COLS  - 1);
assign is_last_l2_col  = (l2_col_idx  == N_L2_COLS  - 1);
assign is_last_wb      = (wb_cnt      == OUT_WORDS   - 1);

// =============================================================================
// Address generation
// =============================================================================
assign w1mem_addr = 32'(l1_col_idx) * W1_LOAD_WORDS + 32'(load_cnt);
assign w2mem_addr = 32'(l2_col_idx) * W2_LOAD_WORDS + 32'(load_cnt);

logic [7:0] x_sub_row;
logic [4:0] x_k_word;
always_comb begin
    x_sub_row = 8'(load_cnt / 24);
    x_k_word  = 5'(load_cnt % 24);
    xmem_addr = (32'(row_grp_idx) * 7 + 32'(x_sub_row)) * 24 + 32'(x_k_word);
end

assign omem_addr = 32'(l2_col_idx) * 3136
                 + 32'(row_grp_idx) * OUT_WORDS
                 + 32'(wb_cnt);

// =============================================================================
// State register
// =============================================================================
always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;

// =============================================================================
// Next-state logic
// =============================================================================
always_comb begin
    next_state = state;
    case (state)
        S_IDLE:        if (start)                              next_state = S_L1_LOAD_X;
        S_L1_LOAD_X:   if (load_cyc == X_LOAD_CYCS  - 1)     next_state = S_L1_SWAP_X;
        S_L1_SWAP_X:                                          next_state = S_L1_LOAD_W;
        S_L1_LOAD_W:   if (load_cyc == W1_LOAD_CYCS - 1)     next_state = S_L1_SWAP_W;
        S_L1_SWAP_W:                                          next_state = S_L1_COMPUTE;
        S_L1_COMPUTE:  if (compute_cnt == N_ACC_L1)           next_state = S_L1_NEXT_COL;
        S_L1_NEXT_COL: if (is_last_l1_col)                   next_state = S_L2_LOAD_W;
                       else                                    next_state = S_L1_LOAD_W;
        S_L2_LOAD_W:   if (load_cyc == W2_LOAD_CYCS - 1)     next_state = S_L2_SWAP_W;
        S_L2_SWAP_W:                                          next_state = S_L2_COMPUTE;
        S_L2_COMPUTE:  if (compute_cnt == N_ACC_L2)           next_state = S_L2_WRITEBACK;
        S_L2_WRITEBACK:if (is_last_wb)                        next_state = S_L2_NEXT_COL;
        S_L2_NEXT_COL: if (is_last_l2_col)                   next_state = S_NEXT_ROW_GRP;
                       else                                    next_state = S_L2_LOAD_W;
        S_NEXT_ROW_GRP:if (is_last_row_grp)                  next_state = S_DONE;
                       else                                    next_state = S_L1_LOAD_X;
        S_DONE:                                               next_state = S_DONE;
        default:                                              next_state = S_IDLE;
    endcase
end

// =============================================================================
// Counter updates
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_grp_idx <= '0; l1_col_idx <= '0; l2_col_idx <= '0;
        load_cyc    <= '0; compute_cnt<= '0; wb_cnt     <= '0;
    end else begin
        case (state)
            S_IDLE, S_L1_SWAP_X, S_L1_SWAP_W, S_L2_SWAP_W,
            S_L1_NEXT_COL, S_L2_NEXT_COL, S_NEXT_ROW_GRP:
                load_cyc <= '0;

            S_L1_LOAD_X:
                load_cyc <= (load_cyc < X_LOAD_CYCS -1) ? load_cyc+1 : '0;
            S_L1_LOAD_W:
                load_cyc <= (load_cyc < W1_LOAD_CYCS-1) ? load_cyc+1 : '0;
            S_L2_LOAD_W:
                load_cyc <= (load_cyc < W2_LOAD_CYCS-1) ? load_cyc+1 : '0;

            S_L1_COMPUTE, S_L2_COMPUTE: begin
                automatic int lim = (state == S_L1_COMPUTE) ? N_ACC_L1 : N_ACC_L2;
                compute_cnt <= (compute_cnt < lim) ? compute_cnt+1 : '0;
            end

            S_L2_WRITEBACK:
                wb_cnt <= (wb_cnt < OUT_WORDS-1) ? wb_cnt+1 : '0;

            S_L1_NEXT_COL: begin
                l1_col_idx  <= (!is_last_l1_col) ? l1_col_idx+1 : '0;
                compute_cnt <= '0;
            end
            S_L2_NEXT_COL: begin
                l2_col_idx  <= (!is_last_l2_col) ? l2_col_idx+1 : '0;
                compute_cnt <= '0;
            end
            S_NEXT_ROW_GRP: begin
                row_grp_idx <= (!is_last_row_grp) ? row_grp_idx+1 : row_grp_idx;
                compute_cnt <= '0;
            end
            default: ;
        endcase
    end
end

// =============================================================================
// sub_cycle generation
// =============================================================================
always_comb begin
    case (state)
        S_L1_COMPUTE: mmu_sub_cycle = (compute_cnt < N_ACC_L1) ?
                                       3'(compute_cnt) : 3'(N_ACC_L1-1);
        S_L2_COMPUTE: mmu_sub_cycle = (compute_cnt < N_ACC_L2) ?
                                       3'(compute_cnt) : 3'(N_ACC_L2-1);
        default:      mmu_sub_cycle = 3'd0;
    endcase
end

// =============================================================================
// Output signal generation
// =============================================================================
assign done = (state == S_DONE);

// Memory read enables
assign w1mem_rd_en = (state == S_L1_LOAD_W) && !is_data_ph;
assign w2mem_rd_en = (state == S_L2_LOAD_W) && !is_data_ph;
assign xmem_rd_en  = (state == S_L1_LOAD_X) && !is_data_ph;

// Weight buffer
assign wbuf_load_en     = ((state == S_L1_LOAD_W) || (state == S_L2_LOAD_W)) && is_data_ph;
assign wbuf_load_k_word = 7'(load_cnt);
assign wbuf_load_data   = (state == S_L1_LOAD_W) ? w1mem_rd_data : w2mem_rd_data;
assign wbuf_swap        = (state == S_L1_SWAP_W) || (state == S_L2_SWAP_W);

// Input buffer (X load)
assign ibuf_load_en      = (state == S_L1_LOAD_X) && is_data_ph;
assign ibuf_load_row     = 3'(x_sub_row[2:0]);
assign ibuf_load_k_word  = {2'b00, x_k_word};
assign ibuf_load_data    = xmem_rd_data;

// ibuf swap:
//   Swap 1 → after X load:     X shadow → active
//   Swap 2 → after last L1 col: Y shadow → active
assign ibuf_swap = (state == S_L1_SWAP_X) ||
                   (state == S_L1_NEXT_COL && is_last_l1_col);

// L1 Y capture into shadow bank
assign ibuf_l1_capture_en = (state == S_L1_COMPUTE) && (compute_cnt == N_ACC_L1);
assign ibuf_l1_col_wr     = l1_col_idx;

// MMU
assign mmu_valid_in = (state == S_L1_COMPUTE) || (state == S_L2_COMPUTE);

always_comb begin
    mmu_op_code = 3'd1;
    mmu_stage   = (state == S_L1_COMPUTE) ? 2'd0 : 2'd2;
end

// L2 output capture
assign l2_capture_en = (state == S_L2_COMPUTE) && (compute_cnt == N_ACC_L2);

// Output buffer / memory
assign obuf_rd_idx = wb_cnt;
assign omem_wr_en  = (state == S_L2_WRITEBACK);

endmodule
