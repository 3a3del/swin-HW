// =============================================================================
// unified_controller.sv
//
// Single FSM that replaces conv_controller, fc_controller, and the DSU.
// All memory addresses, buffer controls, MMU signals, and output buffer
// strobes originate exclusively from this module.
//
// ── Mode selection ────────────────────────────────────────────────────────
//   mode = 0  →  Convolutional engine
//                224×224×3 input  →  56×56×96 output  (4×4 kernel, stride 4)
//   mode = 1  →  MLP engine
//                X(3136×96) × W1(96×384) → Y(3136×384)
//                Y(3136×384) × W2(384×96) → Z(3136×96)
//
// ── Removed blocks ────────────────────────────────────────────────────────
//   conv_controller.sv  ← merged here
//   fc_controller.sv    ← merged here
//   dsu.sv              ← address muxing and W2 base offset folded here;
//                          feedback path mux is a 6-line snippet in top
//
// ── Memory interface convention ───────────────────────────────────────────
//   All memories use 1-cycle registered read latency:
//     cycle N  : addr + rd_en asserted  (is_data_ph == 0)
//     cycle N+1: rd_data captured       (is_data_ph == 1)
//   load_cyc counts both phases: even = address phase, odd = data phase.
//
// ── Conv processing order ─────────────────────────────────────────────────
//   for kernel 0..95
//     for row_group 0..55
//       for chunk 0..7
//         LOAD_W(26) → SWAP_W(1) → LOAD_IMG(168) → SWAP_IMG(1) →
//         COMPUTE(1) → WAIT_OUT(1) → WRITEBACK(7) → NEXT(1)
//   Weights are reloaded only when kernel_idx changes.
//
// ── MLP processing order ──────────────────────────────────────────────────
//   for row_group 0..447
//     LOAD_X(336) → SWAP_X(1)
//     for l1_col 0..383
//       LOAD_W1(48) → SWAP_W1(1) → L1_COMPUTE(3) → L1_NEXT_COL(1)
//     [last L1_NEXT_COL triggers ibuf_swap → Y active]
//     for l2_col 0..95
//       LOAD_W2(192) → SWAP_W2(1) → L2_COMPUTE(9) → WRITEBACK(7) →
//       L2_NEXT_COL(1)
//     NEXT_ROW(1)
// =============================================================================

module unified_controller #(
    parameter int WAW     = 15,   // weight_memory address width
    parameter int FAW     = 17,   // fib_memory address width
    parameter int OAW     = 19,   // output_memory address width
    parameter int W2_BASE = 9216  // word offset of W2 block in weight_memory
)(
    input  logic clk,
    input  logic rst_n,

    // ── Mode and start/done ───────────────────────────────────────────────
    input  logic mode,     // 0 = Conv, 1 = MLP
    input  logic start,
    output logic done,

    // ── Weight memory  (single unified port) ─────────────────────────────
    //   Conv : addr = kernel*13 + word
    //   MLP  : addr = col*W_WORDS + word  (W2 region: addr += W2_BASE)
    output logic [WAW-1:0] wmem_rd_addr,
    output logic           wmem_rd_en,
    input  logic [31:0]    wmem_rd_data,

    // ── Input / feature memory  ───────────────────────────────────────────
    //   Conv : routes to fib_memory (image data)
    //   MLP  : routes to fib_memory (X matrix) OR output_memory.fb
    //          The top-level performs the 2-to-1 mux based on omem_fb_en.
    //          imem_rd_data is the mux output wired back to this port.
    output logic [OAW-1:0] imem_rd_addr,  // OAW covers both fib and omem.fb
    output logic           imem_rd_en,
    input  logic [31:0]    imem_rd_data,  // from fib or omem.fb (top mux)

    // ── Output memory write  ──────────────────────────────────────────────
    output logic [OAW-1:0] omem_wr_addr,
    output logic           omem_wr_en,

    // ── Weight buffer control ─────────────────────────────────────────────
    output logic           wbuf_load_en,
    output logic [3:0]     wbuf_load_pe_idx,    // conv: PE index (0..11)
    output logic [6:0]     wbuf_load_k_word,    // mlp:  k-word index (0..95)
    output logic [31:0]    wbuf_load_data,
    output logic           wbuf_bias_load_en,   // conv only
    output logic [31:0]    wbuf_bias_load_data, // conv only
    output logic           wbuf_swap,

    // ── Input buffer control ──────────────────────────────────────────────
    output logic           ibuf_load_en,
    output logic [3:0]     ibuf_load_pe_idx,    // conv: PE index
    output logic [2:0]     ibuf_load_win_idx,   // conv: window index
    output logic [2:0]     ibuf_load_row,        // mlp:  row index
    output logic [6:0]     ibuf_load_k_word,    // mlp:  k-word index
    output logic [31:0]    ibuf_load_data,
    output logic           ibuf_swap,
    output logic           ibuf_l1_capture_en,  // mlp L1 Y capture
    output logic [8:0]     ibuf_l1_col_wr,      // mlp L1 column being captured

    // ── MMU control ───────────────────────────────────────────────────────
    output logic           mmu_valid_in,
    output logic [2:0]     mmu_op_code,
    output logic [1:0]     mmu_stage,
    output logic [2:0]     mmu_sub_cycle,

    // ── Output buffer control ─────────────────────────────────────────────
    output logic           obuf_capture_en, // strobe to latch mmu_out → obuf
    output logic [2:0]     obuf_rd_idx      // word to read from obuf during WB
);

// =============================================================================
// Parameters
// =============================================================================

// ── Convolutional engine ──────────────────────────────────────────────────
localparam int C_N_KERNELS     = 96;
localparam int C_N_ROW_GROUPS  = 56;   // 224 / stride(4)
localparam int C_N_CHUNKS      = 8;    // 224 / (7 outputs * 4)
localparam int C_WLOAD_WORDS   = 13;   // 12 PE weights + 1 bias
localparam int C_ILOAD_WORDS   = 84;   // 12 PEs × 7 windows
localparam int C_OUT_WORDS     = 7;
localparam int C_WLOAD_CYCS    = C_WLOAD_WORDS * 2;    // 26
localparam int C_ILOAD_CYCS    = C_ILOAD_WORDS * 2;    // 168
localparam int C_IMG_W_WORDS   = 56;                   // 224 / 4
localparam int C_IMG_CH_WORDS  = 56 * 224;             // 12 544
localparam int C_OUT_ROW_WORDS = 56;
localparam int C_OUT_K_WORDS   = 56 * 56;              // 3 136

// ── MLP engine ────────────────────────────────────────────────────────────
localparam int M_N_ROW_GRPS    = 448;  // 3136 / 7
localparam int M_N_L1_COLS     = 384;
localparam int M_N_L2_COLS     = 96;
localparam int M_N_ACC_L1      = 2;    // 96 inputs / (12 PEs × 4 taps)
localparam int M_N_ACC_L2      = 8;    // 384 inputs / (12 PEs × 4 taps)
localparam int M_W1_WORDS      = 24;   // 96 / 4 bytes/word
localparam int M_W2_WORDS      = 96;   // 384 / 4 bytes/word
localparam int M_X_WORDS       = 168;  // 7 rows × 24 words/row
localparam int M_W1_CYCS       = M_W1_WORDS * 2;   // 48
localparam int M_W2_CYCS       = M_W2_WORDS * 2;   // 192
localparam int M_X_CYCS        = M_X_WORDS  * 2;   // 336
localparam int M_OUT_WORDS     = 7;

// =============================================================================
// State encoding
// =============================================================================
typedef enum logic [4:0] {
    S_IDLE          = 5'd0,
    // ── Conv states ──────────────────────────────────────────────────────
    S_C_LOAD_W      = 5'd1,   // load weights from weight_memory
    S_C_SWAP_W      = 5'd2,   // swap weight buffer banks
    S_C_LOAD_IMG    = 5'd3,   // load image tile from fib_memory
    S_C_SWAP_IMG    = 5'd4,   // swap input buffer banks
    S_C_COMPUTE     = 5'd5,   // assert mmu_valid_in, begin accumulation
    S_C_WAIT_OUT    = 5'd6,   // second valid cycle; capture MMU result
    S_C_WRITEBACK   = 5'd7,   // sequentially write 7 outputs to omem
    S_C_NEXT        = 5'd8,   // advance chunk/row/kernel counters
    // ── MLP states ───────────────────────────────────────────────────────
    S_M_L1_LOAD_X   = 5'd9,   // load X row-group from fib_memory
    S_M_L1_SWAP_X   = 5'd10,  // swap input buffer (X → active)
    S_M_L1_LOAD_W   = 5'd11,  // load W1 column from weight_memory
    S_M_L1_SWAP_W   = 5'd12,  // swap weight buffer
    S_M_L1_COMPUTE  = 5'd13,  // L1 MAC (N_ACC_L1+1 cycles)
    S_M_L1_NEXT_COL = 5'd14,  // advance L1 col; on last col swap Y→active
    S_M_L2_LOAD_W   = 5'd15,  // load W2 column from weight_memory (W2 region)
    S_M_L2_SWAP_W   = 5'd16,  // swap weight buffer
    S_M_L2_COMPUTE  = 5'd17,  // L2 MAC (N_ACC_L2+1 cycles)
    S_M_WRITEBACK   = 5'd18,  // sequentially write 7 outputs to omem
    S_M_L2_NEXT_COL = 5'd19,  // advance L2 col
    S_M_NEXT_ROW    = 5'd20,  // advance row group
    S_DONE          = 5'd21
} state_t;

state_t state, next_state;

// =============================================================================
// Counters
// =============================================================================

// Conv counters
logic [6:0] c_kernel_idx;      // 0 .. C_N_KERNELS-1    (7 bits)
logic [5:0] c_row_group_idx;   // 0 .. C_N_ROW_GROUPS-1 (6 bits)
logic [2:0] c_chunk_idx;       // 0 .. C_N_CHUNKS-1     (3 bits)
logic [2:0] c_wb_cnt;          // 0 .. C_OUT_WORDS-1    (3 bits)

// MLP counters
logic [8:0] m_row_grp_idx;     // 0 .. M_N_ROW_GRPS-1  (9 bits, max 447)
logic [8:0] m_l1_col_idx;      // 0 .. M_N_L1_COLS-1   (9 bits, max 383)
logic [6:0] m_l2_col_idx;      // 0 .. M_N_L2_COLS-1   (7 bits)
logic [3:0] m_compute_cnt;     // 0 .. max(N_ACC_L2)    (4 bits, max 8)
logic [2:0] m_wb_cnt;          // 0 .. M_OUT_WORDS-1    (3 bits)

// Shared load-cycle counter.
// Maximum: M_X_CYCS = 336  →  9 bits required.
logic [8:0] load_cyc;
logic       is_data_ph;        // 1 on odd cycles (data capture phase)
logic [7:0] load_cnt;          // word index = load_cyc >> 1
assign is_data_ph = load_cyc[0];
assign load_cnt   = load_cyc[8:1];

// =============================================================================
// Last-element flags
// =============================================================================

// Conv
logic c_last_chunk, c_last_row_group, c_last_kernel;
assign c_last_chunk     = (c_chunk_idx     == C_N_CHUNKS     - 1);
assign c_last_row_group = (c_row_group_idx == C_N_ROW_GROUPS - 1);
assign c_last_kernel    = (c_kernel_idx    == C_N_KERNELS    - 1);

// MLP
logic m_last_row_grp, m_last_l1_col, m_last_l2_col, m_last_wb;
assign m_last_row_grp = (m_row_grp_idx == M_N_ROW_GRPS - 1);
assign m_last_l1_col  = (m_l1_col_idx  == M_N_L1_COLS  - 1);
assign m_last_l2_col  = (m_l2_col_idx  == M_N_L2_COLS  - 1);
assign m_last_wb      = (m_wb_cnt      == M_OUT_WORDS   - 1);

// =============================================================================
// Address generation (all combinatorial)
// =============================================================================

// ── Conv weight ───────────────────────────────────────────────────────────
// layout: kernel_idx * 13 + word  (0..11 = PE weights, 12 = bias)
logic [WAW-1:0] c_wmem_addr;
assign c_wmem_addr = WAW'(c_kernel_idx) * C_WLOAD_WORDS + WAW'(load_cnt);

// ── Conv image ────────────────────────────────────────────────────────────
// PE index maps to (channel, row-within-group):
//   img_ch  = pe_idx[3:2]   (0..2  channels)
//   img_row = row_group*4 + pe_idx[1:0]
// Flat word address in CHW layout:
//   ch * CH_WORDS + row * W_WORDS + chunk * 7 + win
logic [3:0]     c_img_pe;
logic [2:0]     c_img_win;
logic [1:0]     c_img_ch;
logic [7:0]     c_img_row;
logic [OAW-1:0] c_imem_addr;
always_comb begin
    c_img_pe    = 4'(load_cnt / 7);
    c_img_win   = 3'(load_cnt % 7);
    c_img_ch    = c_img_pe[3:2];
    c_img_row   = {2'b0, c_row_group_idx} * 4 + {6'b0, c_img_pe[1:0]};
    c_imem_addr = OAW'(c_img_ch)  * C_IMG_CH_WORDS
                + OAW'(c_img_row) * C_IMG_W_WORDS
                + OAW'(c_chunk_idx) * 7
                + OAW'(c_img_win);
end

// ── Conv output ───────────────────────────────────────────────────────────
// layout: kernel * 3136 + row_group * 56 + chunk * 7 + wb_word
logic [OAW-1:0] c_omem_addr;
assign c_omem_addr = OAW'(c_kernel_idx)    * C_OUT_K_WORDS
                   + OAW'(c_row_group_idx) * C_OUT_ROW_WORDS
                   + OAW'(c_chunk_idx)     * C_OUT_WORDS
                   + OAW'(c_wb_cnt);

// ── MLP W1 ────────────────────────────────────────────────────────────────
// layout: l1_col * 24 + word   (absolute, W1 region starts at 0)
logic [WAW-1:0] m_w1mem_addr;
assign m_w1mem_addr = WAW'(m_l1_col_idx) * M_W1_WORDS + WAW'(load_cnt);

// ── MLP W2 ────────────────────────────────────────────────────────────────
// layout: W2_BASE + l2_col * 96 + word   (absorbs DSU's W2_BASE offset)
logic [WAW-1:0] m_w2mem_addr;
assign m_w2mem_addr = WAW'(W2_BASE)
                    + WAW'(m_l2_col_idx) * M_W2_WORDS
                    + WAW'(load_cnt);

// ── MLP X ─────────────────────────────────────────────────────────────────
// layout (row-major): row * 24 + k_word
//   load_cnt encodes sub_row (0..6) and k_word (0..23):
//     sub_row = load_cnt / 24,  k_word = load_cnt % 24
logic [7:0]     m_x_sub_row;
logic [4:0]     m_x_k_word;
logic [OAW-1:0] m_xmem_addr;
always_comb begin
    m_x_sub_row = 8'(load_cnt / 24);
    m_x_k_word  = 5'(load_cnt % 24);
    m_xmem_addr = (OAW'(m_row_grp_idx) * 7 + OAW'(m_x_sub_row)) * 24
                 + OAW'(m_x_k_word);
end

// ── MLP output ────────────────────────────────────────────────────────────
// layout: l2_col * 3136 + row_grp * 7 + wb_word
logic [OAW-1:0] m_omem_addr;
assign m_omem_addr = OAW'(m_l2_col_idx) * 3136
                   + OAW'(m_row_grp_idx) * M_OUT_WORDS
                   + OAW'(m_wb_cnt);

// =============================================================================
// State register
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;
end

// =============================================================================
// Next-state logic
// =============================================================================
always_comb begin
    next_state = state;
    case (state)

        S_IDLE: begin
            if (start)
                next_state = (mode == 1'b0) ? S_C_LOAD_W : S_M_L1_LOAD_X;
        end

        // ── Conv ─────────────────────────────────────────────────────────
        S_C_LOAD_W:    if (load_cyc == C_WLOAD_CYCS - 1) next_state = S_C_SWAP_W;
        S_C_SWAP_W:                                       next_state = S_C_LOAD_IMG;
        S_C_LOAD_IMG:  if (load_cyc == C_ILOAD_CYCS - 1) next_state = S_C_SWAP_IMG;
        S_C_SWAP_IMG:                                     next_state = S_C_COMPUTE;
        S_C_COMPUTE:                                      next_state = S_C_WAIT_OUT;
        S_C_WAIT_OUT:                                     next_state = S_C_WRITEBACK;
        S_C_WRITEBACK: if (c_wb_cnt == C_OUT_WORDS - 1)  next_state = S_C_NEXT;
        S_C_NEXT: begin
            if (c_last_chunk && c_last_row_group && c_last_kernel)
                next_state = S_DONE;
            else if (c_last_chunk && c_last_row_group)
                next_state = S_C_LOAD_W;   // new kernel → reload weights
            else
                next_state = S_C_LOAD_IMG; // same kernel, new tile
        end

        // ── MLP ──────────────────────────────────────────────────────────
        S_M_L1_LOAD_X:   if (load_cyc == M_X_CYCS  - 1)  next_state = S_M_L1_SWAP_X;
        S_M_L1_SWAP_X:                                    next_state = S_M_L1_LOAD_W;
        S_M_L1_LOAD_W:   if (load_cyc == M_W1_CYCS - 1)  next_state = S_M_L1_SWAP_W;
        S_M_L1_SWAP_W:                                    next_state = S_M_L1_COMPUTE;
        S_M_L1_COMPUTE:  if (m_compute_cnt == M_N_ACC_L1) next_state = S_M_L1_NEXT_COL;
        S_M_L1_NEXT_COL: begin
            if (m_last_l1_col) next_state = S_M_L2_LOAD_W; // all cols done → L2
            else               next_state = S_M_L1_LOAD_W;
        end
        S_M_L2_LOAD_W:   if (load_cyc == M_W2_CYCS - 1)  next_state = S_M_L2_SWAP_W;
        S_M_L2_SWAP_W:                                    next_state = S_M_L2_COMPUTE;
        S_M_L2_COMPUTE:  if (m_compute_cnt == M_N_ACC_L2) next_state = S_M_WRITEBACK;
        S_M_WRITEBACK:   if (m_last_wb)                   next_state = S_M_L2_NEXT_COL;
        S_M_L2_NEXT_COL: begin
            if (m_last_l2_col) next_state = S_M_NEXT_ROW;
            else               next_state = S_M_L2_LOAD_W;
        end
        S_M_NEXT_ROW: begin
            if (m_last_row_grp) next_state = S_DONE;
            else                next_state = S_M_L1_LOAD_X;
        end

        S_DONE:  next_state = S_DONE;
        default: next_state = S_IDLE;

    endcase
end

// =============================================================================
// Counter updates
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        c_kernel_idx    <= '0;
        c_row_group_idx <= '0;
        c_chunk_idx     <= '0;
        c_wb_cnt        <= '0;
        m_row_grp_idx   <= '0;
        m_l1_col_idx    <= '0;
        m_l2_col_idx    <= '0;
        m_compute_cnt   <= '0;
        m_wb_cnt        <= '0;
        load_cyc        <= '0;
    end else begin
        case (state)

            // ── load_cyc resets ────────────────────────────────────────────
            S_IDLE,
            S_C_SWAP_W,  S_C_SWAP_IMG,  S_C_NEXT,
            S_M_L1_SWAP_X, S_M_L1_SWAP_W, S_M_L2_SWAP_W,
            S_M_L1_NEXT_COL, S_M_L2_NEXT_COL, S_M_NEXT_ROW:
                load_cyc <= '0;

            // ── Conv load cycles ───────────────────────────────────────────
            S_C_LOAD_W:
                load_cyc <= (load_cyc < C_WLOAD_CYCS - 1) ? load_cyc + 1 : '0;
            S_C_LOAD_IMG:
                load_cyc <= (load_cyc < C_ILOAD_CYCS - 1) ? load_cyc + 1 : '0;

            // ── Conv writeback ─────────────────────────────────────────────
            S_C_WRITEBACK:
                c_wb_cnt <= (c_wb_cnt < C_OUT_WORDS - 1) ? c_wb_cnt + 1 : '0;

            // ── Conv counter advance ───────────────────────────────────────
            S_C_NEXT: begin
                if (!c_last_chunk) begin
                    c_chunk_idx <= c_chunk_idx + 1;
                end else begin
                    c_chunk_idx <= '0;
                    if (!c_last_row_group)
                        c_row_group_idx <= c_row_group_idx + 1;
                    else begin
                        c_row_group_idx <= '0;
                        if (!c_last_kernel)
                            c_kernel_idx <= c_kernel_idx + 1;
                    end
                end
            end

            // ── MLP load cycles ────────────────────────────────────────────
            S_M_L1_LOAD_X:
                load_cyc <= (load_cyc < M_X_CYCS  - 1) ? load_cyc + 1 : '0;
            S_M_L1_LOAD_W:
                load_cyc <= (load_cyc < M_W1_CYCS - 1) ? load_cyc + 1 : '0;
            S_M_L2_LOAD_W:
                load_cyc <= (load_cyc < M_W2_CYCS - 1) ? load_cyc + 1 : '0;

            // ── MLP compute ────────────────────────────────────────────────
            S_M_L1_COMPUTE:
                m_compute_cnt <= (m_compute_cnt < M_N_ACC_L1) ?
                                  m_compute_cnt + 1 : '0;
            S_M_L2_COMPUTE:
                m_compute_cnt <= (m_compute_cnt < M_N_ACC_L2) ?
                                  m_compute_cnt + 1 : '0;

            // ── MLP writeback ──────────────────────────────────────────────
            S_M_WRITEBACK:
                m_wb_cnt <= (m_wb_cnt < M_OUT_WORDS - 1) ? m_wb_cnt + 1 : '0;

            // ── MLP counter advances ───────────────────────────────────────
            S_M_L1_NEXT_COL: begin
                m_l1_col_idx  <= m_last_l1_col ? '0 : m_l1_col_idx + 1;
                m_compute_cnt <= '0;
            end
            S_M_L2_NEXT_COL: begin
                m_l2_col_idx  <= m_last_l2_col ? '0 : m_l2_col_idx + 1;
                m_compute_cnt <= '0;
            end
            S_M_NEXT_ROW: begin
                m_row_grp_idx <= m_last_row_grp ? m_row_grp_idx : m_row_grp_idx + 1;
                m_l1_col_idx  <= '0;
                m_l2_col_idx  <= '0;
                m_compute_cnt <= '0;
            end

            default: ;
        endcase
    end
end

// =============================================================================
// done
// =============================================================================
assign done = (state == S_DONE);

// =============================================================================
// MMU sub-cycle  (MLP only; 0 in all Conv states)
// =============================================================================
always_comb begin
    case (state)
        S_M_L1_COMPUTE:
            mmu_sub_cycle = (m_compute_cnt < M_N_ACC_L1) ?
                             3'(m_compute_cnt) : 3'(M_N_ACC_L1 - 1);
        S_M_L2_COMPUTE:
            mmu_sub_cycle = (m_compute_cnt < M_N_ACC_L2) ?
                             3'(m_compute_cnt) : 3'(M_N_ACC_L2 - 1);
        default:
            mmu_sub_cycle = 3'd0;
    endcase
end

// =============================================================================
// Weight memory outputs
// =============================================================================
always_comb begin
    wmem_rd_addr = '0;
    wmem_rd_en   = 1'b0;
    case (state)
        S_C_LOAD_W: begin
            wmem_rd_addr = c_wmem_addr;
            wmem_rd_en   = !is_data_ph;
        end
        S_M_L1_LOAD_W: begin
            wmem_rd_addr = m_w1mem_addr;
            wmem_rd_en   = !is_data_ph;
        end
        S_M_L2_LOAD_W: begin
            wmem_rd_addr = m_w2mem_addr;      // already includes W2_BASE
            wmem_rd_en   = !is_data_ph;
        end
        default: ;
    endcase
end

// =============================================================================
// Input / feature memory outputs
// =============================================================================
always_comb begin
    imem_rd_addr = '0;
    imem_rd_en   = 1'b0;
    case (state)
        S_C_LOAD_IMG: begin
            imem_rd_addr = c_imem_addr;
            imem_rd_en   = !is_data_ph;
        end
        S_M_L1_LOAD_X: begin
            imem_rd_addr = m_xmem_addr;
            imem_rd_en   = !is_data_ph;
        end
        default: ;
    endcase
end

// =============================================================================
// Output memory write outputs
// =============================================================================
always_comb begin
    omem_wr_addr = '0;
    omem_wr_en   = 1'b0;
    case (state)
        S_C_WRITEBACK: begin
            omem_wr_addr = c_omem_addr;
            omem_wr_en   = 1'b1;
        end
        S_M_WRITEBACK: begin
            omem_wr_addr = m_omem_addr;
            omem_wr_en   = 1'b1;
        end
        default: ;
    endcase
end

// =============================================================================
// Weight buffer control outputs
// =============================================================================
always_comb begin
    wbuf_load_en        = 1'b0;
    wbuf_load_pe_idx    = '0;
    wbuf_load_k_word    = '0;
    wbuf_load_data      = '0;
    wbuf_bias_load_en   = 1'b0;
    wbuf_bias_load_data = '0;
    wbuf_swap           = 1'b0;

    case (state)
        // ── Conv weight load ───────────────────────────────────────────────
        S_C_LOAD_W: begin
            wbuf_load_en        = is_data_ph && (load_cnt < 12);
            wbuf_load_pe_idx    = 4'(load_cnt);
            wbuf_load_data      = wmem_rd_data;
            wbuf_bias_load_en   = is_data_ph && (load_cnt == 12);
            wbuf_bias_load_data = wmem_rd_data;
        end
        S_C_SWAP_W:
            wbuf_swap = 1'b1;

        // ── MLP W1 load ────────────────────────────────────────────────────
        S_M_L1_LOAD_W: begin
            wbuf_load_en     = is_data_ph;
            wbuf_load_k_word = 7'(load_cnt);
            wbuf_load_data   = wmem_rd_data;
        end
        S_M_L1_SWAP_W:
            wbuf_swap = 1'b1;

        // ── MLP W2 load ────────────────────────────────────────────────────
        S_M_L2_LOAD_W: begin
            wbuf_load_en     = is_data_ph;
            wbuf_load_k_word = 7'(load_cnt);
            wbuf_load_data   = wmem_rd_data;
        end
        S_M_L2_SWAP_W:
            wbuf_swap = 1'b1;

        default: ;
    endcase
end

// =============================================================================
// Input buffer control outputs
// =============================================================================
always_comb begin
    ibuf_load_en       = 1'b0;
    ibuf_load_pe_idx   = '0;
    ibuf_load_win_idx  = '0;
    ibuf_load_row      = '0;
    ibuf_load_k_word   = '0;
    ibuf_load_data     = '0;
    ibuf_swap          = 1'b0;
    ibuf_l1_capture_en = 1'b0;
    ibuf_l1_col_wr     = '0;

    case (state)
        // ── Conv image load ────────────────────────────────────────────────
        S_C_LOAD_IMG: begin
            ibuf_load_en      = is_data_ph;
            ibuf_load_pe_idx  = c_img_pe;
            ibuf_load_win_idx = c_img_win;
            ibuf_load_data    = imem_rd_data;
        end
        S_C_SWAP_IMG:
            ibuf_swap = 1'b1;

        // ── MLP X load ─────────────────────────────────────────────────────
        S_M_L1_LOAD_X: begin
            ibuf_load_en     = is_data_ph;
            ibuf_load_row    = 3'(m_x_sub_row[2:0]);
            ibuf_load_k_word = {2'b00, m_x_k_word};
            ibuf_load_data   = imem_rd_data;
        end

        // Swap 1: X shadow → active after loading
        S_M_L1_SWAP_X:
            ibuf_swap = 1'b1;

        // L1 compute: capture Y into shadow bank on final sub-cycle
        S_M_L1_COMPUTE: begin
            ibuf_l1_capture_en = (m_compute_cnt == M_N_ACC_L1);
            ibuf_l1_col_wr     = m_l1_col_idx;
        end

        // Swap 2: after last L1 col → Y shadow becomes active for L2
        S_M_L1_NEXT_COL:
            ibuf_swap = m_last_l1_col;

        default: ;
    endcase
end

// =============================================================================
// MMU control outputs
// =============================================================================
always_comb begin
    mmu_valid_in = 1'b0;
    mmu_op_code  = 3'd0;
    mmu_stage    = 2'd0;

    case (state)
        // Conv: two-cycle valid window; op_code = 0 (pass-through in valid_ctrl)
        S_C_COMPUTE,
        S_C_WAIT_OUT: begin
            mmu_valid_in = 1'b1;
            mmu_op_code  = 3'd0;
            mmu_stage    = 2'd0;
        end
        // MLP L1: op_code = 1, stage = 0  →  count_mod = 2 sub-cycles
        S_M_L1_COMPUTE: begin
            mmu_valid_in = 1'b1;
            mmu_op_code  = 3'd1;
            mmu_stage    = 2'd0;
        end
        // MLP L2: op_code = 1, stage = 2  →  count_mod = 8 sub-cycles
        S_M_L2_COMPUTE: begin
            mmu_valid_in = 1'b1;
            mmu_op_code  = 3'd1;
            mmu_stage    = 2'd2;
        end
        default: ;
    endcase
end

// =============================================================================
// Output buffer control outputs
// =============================================================================
always_comb begin
    obuf_capture_en = 1'b0;
    obuf_rd_idx     = '0;

    case (state)
        // Conv: capture MMU result during WAIT_OUT; read during WRITEBACK
        S_C_WAIT_OUT:
            obuf_capture_en = 1'b1;
        S_C_WRITEBACK:
            obuf_rd_idx = c_wb_cnt;

        // MLP L2: capture on final compute sub-cycle; read during WRITEBACK
        S_M_L2_COMPUTE:
            obuf_capture_en = (m_compute_cnt == M_N_ACC_L2);
        S_M_WRITEBACK:
            obuf_rd_idx = m_wb_cnt;

        default: ;
    endcase
end

endmodule
