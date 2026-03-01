// =============================================================================
// unified_controller.sv  (rev 2 — double-buffered weight memory)
//
// ── What changed from rev 1 ───────────────────────────────────────────────
//   The weight_memory is now double-buffered (active bank / shadow bank).
//   This controller is the SOLE source of ALL weight-memory control signals:
//
//     wmem_swap           — swap active↔shadow after a shadow fill completes
//     wmem_shadow_wr_addr — address driven into shadow bank each data phase
//     wmem_shadow_wr_en   — write-enable for shadow bank
//     ext_weight_rd_addr  — address sent to the external big memory
//     ext_weight_rd_en    — read-enable for external big memory
//
//   The external big memory is assumed to have 1-cycle read latency (matching
//   all other memories).  Its rd_data output register is wired directly to
//   weight_memory.shadow_wr_data in full_system_top.
//
// ── Double-buffer prefetch mechanism ─────────────────────────────────────
//
//   ALL accesses use identical 2-phase timing (address phase / data phase):
//     !is_data_ph  →  assert ext_weight_rd_en + ext_weight_rd_addr
//     is_data_ph   →  assert wmem_shadow_wr_en + wmem_shadow_wr_addr
//
//   This keeps the shadow-write cycle count identical to the active-read
//   cycle count in every LOAD_W state, achieving PERFECT cycle-for-cycle
//   overlap between serving the current weights and prefetching the next.
//
//   Phase A — Init preload (new states, once per weight-set debut):
//     S_C_INIT_PRELOAD   fills shadow with kernel-0   (26 cyc = C_WLOAD_CYCS)
//     S_M_L1_PRELOAD0    fills shadow with W1-col-0   (48 cyc = M_W1_CYCS)
//                        entered from both S_IDLE and S_M_NEXT_ROW
//     S_M_L2_PRELOAD0    fills shadow with W2-col-0   (192 cyc = M_W2_CYCS)
//                        entered after the last L1 column every row group
//   Each init-preload is followed by a 1-cycle *_WMEM_SWAP state.
//
//   Phase B — Overlapped preload (steady state):
//     During S_C_LOAD_W   : preload kernel(k+1)  into shadow  (guard: !last_k)
//     During S_M_L1_LOAD_W: preload W1-col(c+1) into shadow  (guard: !last_c)
//     During S_M_L2_LOAD_W: preload W2-col(c+1) into shadow  (guard: !last_c)
//   Each LOAD_W is immediately followed by a SWAP_W state that asserts BOTH
//   wbuf_swap and wmem_swap, making the freshly-filled shadow bank active.
//
// ── State encoding (28 states, 5-bit) ────────────────────────────────────
//
//   S_IDLE              =  0
//   S_C_INIT_PRELOAD    =  1  NEW  fill shadow with kernel-0
//   S_C_INIT_WMEM_SWAP  =  2  NEW  swap wmem (kernel-0 → active)
//   S_C_LOAD_W          =  3       read active→wbuf  +  shadow←kernel(k+1)
//   S_C_SWAP_W          =  4       swap wbuf  +  swap wmem
//   S_C_LOAD_IMG        =  5
//   S_C_SWAP_IMG        =  6
//   S_C_COMPUTE         =  7
//   S_C_WAIT_OUT        =  8
//   S_C_WRITEBACK       =  9
//   S_C_NEXT            = 10
//   S_M_L1_PRELOAD0     = 11  NEW  fill shadow with W1-col-0  (per row grp)
//   S_M_L1_WMEM_SWAP0   = 12  NEW  swap wmem (W1-col-0 → active)
//   S_M_L1_LOAD_X       = 13
//   S_M_L1_SWAP_X       = 14
//   S_M_L1_LOAD_W       = 15       read active→wbuf  +  shadow←W1-col(c+1)
//   S_M_L1_SWAP_W       = 16       swap wbuf  +  swap wmem
//   S_M_L1_COMPUTE      = 17
//   S_M_L1_NEXT_COL     = 18
//   S_M_L2_PRELOAD0     = 19  NEW  fill shadow with W2-col-0  (per row grp)
//   S_M_L2_WMEM_SWAP0   = 20  NEW  swap wmem (W2-col-0 → active)
//   S_M_L2_LOAD_W       = 21       read active→wbuf  +  shadow←W2-col(c+1)
//   S_M_L2_SWAP_W       = 22       swap wbuf  +  swap wmem
//   S_M_L2_COMPUTE      = 23
//   S_M_WRITEBACK       = 24
//   S_M_L2_NEXT_COL     = 25
//   S_M_NEXT_ROW        = 26
//   S_DONE              = 27
// =============================================================================

module unified_controller #(
    parameter int WAW     = 15,    // weight_memory address width
    parameter int FAW     = 17,    // fib_memory address width
    parameter int OAW     = 19,    // output_memory address width
    parameter int W2_BASE = 9216   // word offset of W2 block in weight_memory
)(
    input  logic clk,
    input  logic rst_n,

    // ── Mode and start/done ───────────────────────────────────────────────
    input  logic mode,
    input  logic start,
    output logic done,

    // ── Weight memory — active bank read  ────────────────────────────────
    output logic [WAW-1:0] wmem_rd_addr,
    output logic           wmem_rd_en,
    input  logic [31:0]    wmem_rd_data,

    // ── Weight memory — shadow bank write ────────────────────────────────
    //    addr + en driven here; data wired in top from ext_weight_rd_data
    output logic [WAW-1:0] wmem_shadow_wr_addr,
    output logic           wmem_shadow_wr_en,

    // ── Weight memory — bank swap ─────────────────────────────────────────
    output logic           wmem_swap,

    // ── External big memory (weight source, 1-cycle latency) ─────────────
    //    rd_data feeds weight_memory.shadow_wr_data directly in top
    output logic [WAW-1:0] ext_weight_rd_addr,
    output logic           ext_weight_rd_en,

    // ── Input / feature memory (fib or omem.fb, muxed in top) ────────────
    output logic [OAW-1:0] imem_rd_addr,
    output logic           imem_rd_en,
    input  logic [31:0]    imem_rd_data,

    // ── Output memory write ───────────────────────────────────────────────
    output logic [OAW-1:0] omem_wr_addr,
    output logic           omem_wr_en,

    // ── Weight buffer control ─────────────────────────────────────────────
    output logic           wbuf_load_en,
    output logic [3:0]     wbuf_load_pe_idx,    // conv: PE index 0..11
    output logic [6:0]     wbuf_load_k_word,    // mlp:  k-word index 0..95
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
    output logic [8:0]     ibuf_l1_col_wr,

    // ── MMU control ───────────────────────────────────────────────────────
    output logic           mmu_valid_in,
    output logic [2:0]     mmu_op_code,
    output logic [1:0]     mmu_stage,
    output logic [2:0]     mmu_sub_cycle,

    // ── Output buffer control ─────────────────────────────────────────────
    output logic           obuf_capture_en,
    output logic [2:0]     obuf_rd_idx
);

// =============================================================================
// Parameters
// =============================================================================

// ── Convolutional engine ──────────────────────────────────────────────────
localparam int C_N_KERNELS     = 96;
localparam int C_N_ROW_GROUPS  = 56;
localparam int C_N_CHUNKS      = 8;
localparam int C_WLOAD_WORDS   = 13;    // 12 PE weights + 1 bias
localparam int C_ILOAD_WORDS   = 84;    // 12 PEs × 7 windows
localparam int C_OUT_WORDS     = 7;
localparam int C_WLOAD_CYCS    = C_WLOAD_WORDS * 2;    // 26
localparam int C_ILOAD_CYCS    = C_ILOAD_WORDS * 2;    // 168
localparam int C_IMG_W_WORDS   = 56;
localparam int C_IMG_CH_WORDS  = 56 * 224;
localparam int C_OUT_ROW_WORDS = 56;
localparam int C_OUT_K_WORDS   = 56 * 56;

// ── MLP engine ────────────────────────────────────────────────────────────
localparam int M_N_ROW_GRPS    = 448;
localparam int M_N_L1_COLS     = 384;
localparam int M_N_L2_COLS     = 96;
localparam int M_N_ACC_L1      = 2;
localparam int M_N_ACC_L2      = 8;
localparam int M_W1_WORDS      = 24;
localparam int M_W2_WORDS      = 96;
localparam int M_X_WORDS       = 168;
localparam int M_W1_CYCS       = M_W1_WORDS * 2;   // 48
localparam int M_W2_CYCS       = M_W2_WORDS * 2;   // 192
localparam int M_X_CYCS        = M_X_WORDS  * 2;   // 336
localparam int M_OUT_WORDS     = 7;

// =============================================================================
// State encoding  (28 states, 5-bit)
// =============================================================================
typedef enum logic [4:0] {
    S_IDLE              = 5'd0,
    S_C_INIT_PRELOAD    = 5'd1,
    S_C_INIT_WMEM_SWAP  = 5'd2,
    S_C_LOAD_W          = 5'd3,
    S_C_SWAP_W          = 5'd4,
    S_C_LOAD_IMG        = 5'd5,
    S_C_SWAP_IMG        = 5'd6,
    S_C_COMPUTE         = 5'd7,
    S_C_WAIT_OUT        = 5'd8,
    S_C_WRITEBACK       = 5'd9,
    S_C_NEXT            = 5'd10,
    S_M_L1_PRELOAD0     = 5'd11,
    S_M_L1_WMEM_SWAP0   = 5'd12,
    S_M_L1_LOAD_X       = 5'd13,
    S_M_L1_SWAP_X       = 5'd14,
    S_M_L1_LOAD_W       = 5'd15,
    S_M_L1_SWAP_W       = 5'd16,
    S_M_L1_COMPUTE      = 5'd17,
    S_M_L1_NEXT_COL     = 5'd18,
    S_M_L2_PRELOAD0     = 5'd19,
    S_M_L2_WMEM_SWAP0   = 5'd20,
    S_M_L2_LOAD_W       = 5'd21,
    S_M_L2_SWAP_W       = 5'd22,
    S_M_L2_COMPUTE      = 5'd23,
    S_M_WRITEBACK       = 5'd24,
    S_M_L2_NEXT_COL     = 5'd25,
    S_M_NEXT_ROW        = 5'd26,
    S_DONE              = 5'd27
} state_t;

state_t state, next_state;

// =============================================================================
// Counters
// =============================================================================

// Conv
logic [6:0] c_kernel_idx;
logic [5:0] c_row_group_idx;
logic [2:0] c_chunk_idx;
logic [2:0] c_wb_cnt;

// MLP
logic [8:0] m_row_grp_idx;     // max 447
logic [8:0] m_l1_col_idx;      // max 383
logic [6:0] m_l2_col_idx;
logic [3:0] m_compute_cnt;
logic [2:0] m_wb_cnt;

// Shared load cycle (max M_X_CYCS = 336 → 9 bits)
logic [8:0] load_cyc;
logic       is_data_ph;        // 1 on odd (data) cycles
logic [7:0] load_cnt;          // word index = load_cyc >> 1
assign is_data_ph = load_cyc[0];
assign load_cnt   = load_cyc[8:1];

// =============================================================================
// Last-element flags
// =============================================================================
logic c_last_chunk, c_last_row_group, c_last_kernel;
assign c_last_chunk     = (c_chunk_idx     == C_N_CHUNKS     - 1);
assign c_last_row_group = (c_row_group_idx == C_N_ROW_GROUPS - 1);
assign c_last_kernel    = (c_kernel_idx    == C_N_KERNELS    - 1);

logic m_last_row_grp, m_last_l1_col, m_last_l2_col, m_last_wb;
assign m_last_row_grp = (m_row_grp_idx == M_N_ROW_GRPS - 1);
assign m_last_l1_col  = (m_l1_col_idx  == M_N_L1_COLS  - 1);
assign m_last_l2_col  = (m_l2_col_idx  == M_N_L2_COLS  - 1);
assign m_last_wb      = (m_wb_cnt      == M_OUT_WORDS   - 1);

// =============================================================================
// Address generation (combinatorial)
// =============================================================================

// ── Conv weight (active bank) ─────────────────────────────────────────────
logic [WAW-1:0] c_wmem_rd_addr;
assign c_wmem_rd_addr = WAW'(c_kernel_idx) * C_WLOAD_WORDS + WAW'(load_cnt);

// ── Conv image ────────────────────────────────────────────────────────────
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
    c_imem_addr = OAW'(c_img_ch)    * C_IMG_CH_WORDS
                + OAW'(c_img_row)   * C_IMG_W_WORDS
                + OAW'(c_chunk_idx) * 7
                + OAW'(c_img_win);
end

// ── Conv output ───────────────────────────────────────────────────────────
logic [OAW-1:0] c_omem_addr;
assign c_omem_addr = OAW'(c_kernel_idx)    * C_OUT_K_WORDS
                   + OAW'(c_row_group_idx) * C_OUT_ROW_WORDS
                   + OAW'(c_chunk_idx)     * C_OUT_WORDS
                   + OAW'(c_wb_cnt);

// ── MLP W1 (active bank) ──────────────────────────────────────────────────
logic [WAW-1:0] m_w1_rd_addr;
assign m_w1_rd_addr = WAW'(m_l1_col_idx) * M_W1_WORDS + WAW'(load_cnt);

// ── MLP W2 (active bank, W2_BASE included) ────────────────────────────────
logic [WAW-1:0] m_w2_rd_addr;
assign m_w2_rd_addr = WAW'(W2_BASE)
                    + WAW'(m_l2_col_idx) * M_W2_WORDS
                    + WAW'(load_cnt);

// ── MLP X ─────────────────────────────────────────────────────────────────
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
logic [OAW-1:0] m_omem_addr;
assign m_omem_addr = OAW'(m_l2_col_idx) * 3136
                   + OAW'(m_row_grp_idx) * M_OUT_WORDS
                   + OAW'(m_wb_cnt);

// ── Shadow write addresses (next-set helpers) ─────────────────────────────
// Used by overlapped preload inside LOAD_W states.
// Each is guarded by !last_* at the usage site.
logic [WAW-1:0] c_shadow_next_addr;
assign c_shadow_next_addr = WAW'(c_kernel_idx + 1) * C_WLOAD_WORDS
                          + WAW'(load_cnt);

logic [WAW-1:0] m_shadow_w1_next_addr;
assign m_shadow_w1_next_addr = WAW'(m_l1_col_idx + 1) * M_W1_WORDS
                             + WAW'(load_cnt);

logic [WAW-1:0] m_shadow_w2_next_addr;
assign m_shadow_w2_next_addr = WAW'(W2_BASE)
                             + WAW'(m_l2_col_idx + 1) * M_W2_WORDS
                             + WAW'(load_cnt);

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
                next_state = (mode == 1'b0) ? S_C_INIT_PRELOAD
                                            : S_M_L1_PRELOAD0;
        end

        // ── Conv ──────────────────────────────────────────────────────────
        S_C_INIT_PRELOAD:
            if (load_cyc == C_WLOAD_CYCS - 1) next_state = S_C_INIT_WMEM_SWAP;
        S_C_INIT_WMEM_SWAP:                    next_state = S_C_LOAD_W;

        S_C_LOAD_W:
            if (load_cyc == C_WLOAD_CYCS - 1) next_state = S_C_SWAP_W;
        S_C_SWAP_W:                            next_state = S_C_LOAD_IMG;

        S_C_LOAD_IMG:
            if (load_cyc == C_ILOAD_CYCS - 1) next_state = S_C_SWAP_IMG;
        S_C_SWAP_IMG:                          next_state = S_C_COMPUTE;
        S_C_COMPUTE:                           next_state = S_C_WAIT_OUT;
        S_C_WAIT_OUT:                          next_state = S_C_WRITEBACK;
        S_C_WRITEBACK:
            if (c_wb_cnt == C_OUT_WORDS - 1)   next_state = S_C_NEXT;
        S_C_NEXT: begin
            if      (c_last_chunk && c_last_row_group && c_last_kernel)
                next_state = S_DONE;
            else if (c_last_chunk && c_last_row_group)
                next_state = S_C_LOAD_W;    // new kernel (shadow already filled)
            else
                next_state = S_C_LOAD_IMG;  // same kernel, new tile
        end

        // ── MLP ───────────────────────────────────────────────────────────
        S_M_L1_PRELOAD0:
            if (load_cyc == M_W1_CYCS - 1)    next_state = S_M_L1_WMEM_SWAP0;
        S_M_L1_WMEM_SWAP0:                     next_state = S_M_L1_LOAD_X;

        S_M_L1_LOAD_X:
            if (load_cyc == M_X_CYCS  - 1)    next_state = S_M_L1_SWAP_X;
        S_M_L1_SWAP_X:                         next_state = S_M_L1_LOAD_W;

        S_M_L1_LOAD_W:
            if (load_cyc == M_W1_CYCS - 1)    next_state = S_M_L1_SWAP_W;
        S_M_L1_SWAP_W:                         next_state = S_M_L1_COMPUTE;

        S_M_L1_COMPUTE:
            if (m_compute_cnt == M_N_ACC_L1)   next_state = S_M_L1_NEXT_COL;
        S_M_L1_NEXT_COL: begin
            if (m_last_l1_col) next_state = S_M_L2_PRELOAD0;  // → W2 init
            else               next_state = S_M_L1_LOAD_W;
        end

        S_M_L2_PRELOAD0:
            if (load_cyc == M_W2_CYCS - 1)    next_state = S_M_L2_WMEM_SWAP0;
        S_M_L2_WMEM_SWAP0:                     next_state = S_M_L2_LOAD_W;

        S_M_L2_LOAD_W:
            if (load_cyc == M_W2_CYCS - 1)    next_state = S_M_L2_SWAP_W;
        S_M_L2_SWAP_W:                         next_state = S_M_L2_COMPUTE;

        S_M_L2_COMPUTE:
            if (m_compute_cnt == M_N_ACC_L2)   next_state = S_M_WRITEBACK;
        S_M_WRITEBACK:
            if (m_last_wb)                     next_state = S_M_L2_NEXT_COL;
        S_M_L2_NEXT_COL: begin
            if (m_last_l2_col) next_state = S_M_NEXT_ROW;
            else               next_state = S_M_L2_LOAD_W;
        end
        S_M_NEXT_ROW: begin
            if (m_last_row_grp) next_state = S_DONE;
            else                next_state = S_M_L1_PRELOAD0;  // re-preload W1 col0
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
        c_kernel_idx    <= '0; c_row_group_idx <= '0;
        c_chunk_idx     <= '0; c_wb_cnt        <= '0;
        m_row_grp_idx   <= '0; m_l1_col_idx    <= '0;
        m_l2_col_idx    <= '0; m_compute_cnt   <= '0;
        m_wb_cnt        <= '0; load_cyc        <= '0;
    end else begin
        case (state)

            // ── load_cyc resets ─────────────────────────────────────────────
            S_IDLE,
            S_C_INIT_WMEM_SWAP, S_C_SWAP_W,   S_C_SWAP_IMG,  S_C_NEXT,
            S_M_L1_WMEM_SWAP0,  S_M_L1_SWAP_X, S_M_L1_SWAP_W, S_M_L1_NEXT_COL,
            S_M_L2_WMEM_SWAP0,  S_M_L2_SWAP_W, S_M_L2_NEXT_COL, S_M_NEXT_ROW:
                load_cyc <= '0;

            // ── Conv init preload ────────────────────────────────────────────
            S_C_INIT_PRELOAD:
                load_cyc <= (load_cyc < C_WLOAD_CYCS - 1) ? load_cyc + 1 : '0;

            // ── Conv LOAD_W and LOAD_IMG ─────────────────────────────────────
            S_C_LOAD_W:
                load_cyc <= (load_cyc < C_WLOAD_CYCS - 1) ? load_cyc + 1 : '0;
            S_C_LOAD_IMG:
                load_cyc <= (load_cyc < C_ILOAD_CYCS - 1) ? load_cyc + 1 : '0;

            // ── Conv writeback and advance ───────────────────────────────────
            S_C_WRITEBACK:
                c_wb_cnt <= (c_wb_cnt < C_OUT_WORDS - 1) ? c_wb_cnt + 1 : '0;
            S_C_NEXT: begin
                if (!c_last_chunk)
                    c_chunk_idx <= c_chunk_idx + 1;
                else begin
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

            // ── MLP L1 init preload ──────────────────────────────────────────
            S_M_L1_PRELOAD0:
                load_cyc <= (load_cyc < M_W1_CYCS - 1) ? load_cyc + 1 : '0;

            // ── MLP X load ───────────────────────────────────────────────────
            S_M_L1_LOAD_X:
                load_cyc <= (load_cyc < M_X_CYCS  - 1) ? load_cyc + 1 : '0;

            // ── MLP W1 load ──────────────────────────────────────────────────
            S_M_L1_LOAD_W:
                load_cyc <= (load_cyc < M_W1_CYCS - 1) ? load_cyc + 1 : '0;

            // ── MLP L1 compute and advance ───────────────────────────────────
            S_M_L1_COMPUTE:
                m_compute_cnt <= (m_compute_cnt < M_N_ACC_L1) ?
                                  m_compute_cnt + 1 : '0;
            S_M_L1_NEXT_COL: begin
                m_l1_col_idx  <= m_last_l1_col ? '0 : m_l1_col_idx + 1;
                m_compute_cnt <= '0;
            end

            // ── MLP L2 init preload ──────────────────────────────────────────
            S_M_L2_PRELOAD0:
                load_cyc <= (load_cyc < M_W2_CYCS - 1) ? load_cyc + 1 : '0;

            // ── MLP W2 load ──────────────────────────────────────────────────
            S_M_L2_LOAD_W:
                load_cyc <= (load_cyc < M_W2_CYCS - 1) ? load_cyc + 1 : '0;

            // ── MLP L2 compute, writeback, and advance ───────────────────────
            S_M_L2_COMPUTE:
                m_compute_cnt <= (m_compute_cnt < M_N_ACC_L2) ?
                                  m_compute_cnt + 1 : '0;
            S_M_WRITEBACK:
                m_wb_cnt <= (m_wb_cnt < M_OUT_WORDS - 1) ? m_wb_cnt + 1 : '0;
            S_M_L2_NEXT_COL: begin
                m_l2_col_idx  <= m_last_l2_col ? '0 : m_l2_col_idx + 1;
                m_compute_cnt <= '0;
            end

            // ── MLP row group advance ────────────────────────────────────────
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
// MMU sub-cycle
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
// Weight memory — active bank read
// =============================================================================
always_comb begin
    wmem_rd_addr = '0;
    wmem_rd_en   = 1'b0;
    case (state)
        S_C_LOAD_W: begin
            wmem_rd_addr = c_wmem_rd_addr;
            wmem_rd_en   = !is_data_ph;
        end
        S_M_L1_LOAD_W: begin
            wmem_rd_addr = m_w1_rd_addr;
            wmem_rd_en   = !is_data_ph;
        end
        S_M_L2_LOAD_W: begin
            wmem_rd_addr = m_w2_rd_addr;
            wmem_rd_en   = !is_data_ph;
        end
        default: ;
    endcase
end

// =============================================================================
// Weight memory — shadow bank write  +  external big memory read
//
// Timing:  both the active-bank read and the shadow-bank write use the same
//          2-phase (address / data) convention so they interlock perfectly:
//
//   !is_data_ph :  ext_weight_rd_en=1, ext_weight_rd_addr = target address
//    is_data_ph :  wmem_shadow_wr_en=1, wmem_shadow_wr_addr = same target
//                  (ext_weight_rd_data arrives this cycle, wired in top)
//
// Init-preload states fill the shadow bank unconditionally.
// Overlapped LOAD_W states are guarded by !last_* so we don't clobber the
// shadow bank when there is no next set to fetch.
//
// wmem_swap is asserted for exactly one cycle in each *_WMEM_SWAP* or
// SWAP_W state, atomically promoting shadow → active.
// =============================================================================
always_comb begin
    ext_weight_rd_addr  = '0;
    ext_weight_rd_en    = 1'b0;
    wmem_shadow_wr_addr = '0;
    wmem_shadow_wr_en   = 1'b0;
    wmem_swap           = 1'b0;

    case (state)

        // ── Conv init preload: kernel 0 (addr = 0..12) ────────────────────
        S_C_INIT_PRELOAD: begin
            ext_weight_rd_en    = !is_data_ph;
            ext_weight_rd_addr  = WAW'(load_cnt);
            wmem_shadow_wr_en   = is_data_ph;
            wmem_shadow_wr_addr = WAW'(load_cnt);
        end
        S_C_INIT_WMEM_SWAP:
            wmem_swap = 1'b1;

        // ── Conv LOAD_W: preload kernel k+1 in parallel ────────────────────
        // Both the active-bank read (above) and this shadow write
        // use load_cnt for their word index, so they march in lockstep.
        S_C_LOAD_W: begin
            if (!c_last_kernel) begin
                ext_weight_rd_en    = !is_data_ph;
                ext_weight_rd_addr  = c_shadow_next_addr;
                wmem_shadow_wr_en   = is_data_ph;
                wmem_shadow_wr_addr = c_shadow_next_addr;
            end
        end
        S_C_SWAP_W:
            wmem_swap = 1'b1;   // always swap (harmless when no preload occurred)

        // ── MLP L1 init preload: W1 col 0 (addr = 0..23) ─────────────────
        S_M_L1_PRELOAD0: begin
            ext_weight_rd_en    = !is_data_ph;
            ext_weight_rd_addr  = WAW'(load_cnt);
            wmem_shadow_wr_en   = is_data_ph;
            wmem_shadow_wr_addr = WAW'(load_cnt);
        end
        S_M_L1_WMEM_SWAP0:
            wmem_swap = 1'b1;

        // ── MLP L1 LOAD_W: preload W1 col c+1 in parallel ─────────────────
        S_M_L1_LOAD_W: begin
            if (!m_last_l1_col) begin
                ext_weight_rd_en    = !is_data_ph;
                ext_weight_rd_addr  = m_shadow_w1_next_addr;
                wmem_shadow_wr_en   = is_data_ph;
                wmem_shadow_wr_addr = m_shadow_w1_next_addr;
            end
        end
        S_M_L1_SWAP_W:
            wmem_swap = 1'b1;

        // ── MLP L2 init preload: W2 col 0 (addr = W2_BASE..W2_BASE+95) ────
        S_M_L2_PRELOAD0: begin
            ext_weight_rd_en    = !is_data_ph;
            ext_weight_rd_addr  = WAW'(W2_BASE) + WAW'(load_cnt);
            wmem_shadow_wr_en   = is_data_ph;
            wmem_shadow_wr_addr = WAW'(W2_BASE) + WAW'(load_cnt);
        end
        S_M_L2_WMEM_SWAP0:
            wmem_swap = 1'b1;

        // ── MLP L2 LOAD_W: preload W2 col c+1 in parallel ─────────────────
        S_M_L2_LOAD_W: begin
            if (!m_last_l2_col) begin
                ext_weight_rd_en    = !is_data_ph;
                ext_weight_rd_addr  = m_shadow_w2_next_addr;
                wmem_shadow_wr_en   = is_data_ph;
                wmem_shadow_wr_addr = m_shadow_w2_next_addr;
            end
        end
        S_M_L2_SWAP_W:
            wmem_swap = 1'b1;

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
        S_C_LOAD_W: begin
            wbuf_load_en        = is_data_ph && (load_cnt < 12);
            wbuf_load_pe_idx    = 4'(load_cnt);
            wbuf_load_data      = wmem_rd_data;
            wbuf_bias_load_en   = is_data_ph && (load_cnt == 12);
            wbuf_bias_load_data = wmem_rd_data;
        end
        S_C_SWAP_W: wbuf_swap = 1'b1;

        S_M_L1_LOAD_W: begin
            wbuf_load_en     = is_data_ph;
            wbuf_load_k_word = 7'(load_cnt);
            wbuf_load_data   = wmem_rd_data;
        end
        S_M_L1_SWAP_W: wbuf_swap = 1'b1;

        S_M_L2_LOAD_W: begin
            wbuf_load_en     = is_data_ph;
            wbuf_load_k_word = 7'(load_cnt);
            wbuf_load_data   = wmem_rd_data;
        end
        S_M_L2_SWAP_W: wbuf_swap = 1'b1;

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
        S_C_LOAD_IMG: begin
            ibuf_load_en      = is_data_ph;
            ibuf_load_pe_idx  = c_img_pe;
            ibuf_load_win_idx = c_img_win;
            ibuf_load_data    = imem_rd_data;
        end
        S_C_SWAP_IMG: ibuf_swap = 1'b1;

        S_M_L1_LOAD_X: begin
            ibuf_load_en     = is_data_ph;
            ibuf_load_row    = 3'(m_x_sub_row[2:0]);
            ibuf_load_k_word = {2'b00, m_x_k_word};
            ibuf_load_data   = imem_rd_data;
        end
        S_M_L1_SWAP_X: ibuf_swap = 1'b1;

        S_M_L1_COMPUTE: begin
            ibuf_l1_capture_en = (m_compute_cnt == M_N_ACC_L1);
            ibuf_l1_col_wr     = m_l1_col_idx;
        end

        S_M_L1_NEXT_COL:
            ibuf_swap = m_last_l1_col;  // Y shadow → active after last L1 col

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
        S_C_COMPUTE,
        S_C_WAIT_OUT: begin
            mmu_valid_in = 1'b1;
            mmu_op_code  = 3'd0;
            mmu_stage    = 2'd0;
        end
        S_M_L1_COMPUTE: begin
            mmu_valid_in = 1'b1;
            mmu_op_code  = 3'd1;
            mmu_stage    = 2'd0;
        end
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
        S_C_WAIT_OUT:   obuf_capture_en = 1'b1;
        S_C_WRITEBACK:  obuf_rd_idx = c_wb_cnt;

        S_M_L2_COMPUTE: obuf_capture_en = (m_compute_cnt == M_N_ACC_L2);
        S_M_WRITEBACK:  obuf_rd_idx = m_wb_cnt;

        default: ;
    endcase
end

endmodule