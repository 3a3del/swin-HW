// =============================================================================
// full_system_top.sv  (rev 5 — unified controller)
//
// ── What changed from rev 4 ───────────────────────────────────────────────
//   Removed instances:  u_conv_ctrl, u_fc_ctrl, u_dsu
//   Added instance:     u_ctrl  (unified_controller)
//
//   The unified_controller is the single source of truth for every control
//   signal in the system: memory addresses/enables, buffer load/swap strobes,
//   MMU valid/op/stage/sub-cycle, and output-buffer capture/index.
//
//   The DSU's only non-trivial logic — the 2-to-1 mux that routes controller
//   input-memory requests to either fib_memory (normal) or output_memory.fb
//   (feedback) — is now expressed as a ~10-line combinatorial block directly
//   in this file.  No separate module is needed.
//
// ── Signal flow (unchanged from rev 4) ───────────────────────────────────
//
//   output_buffer
//        │  obuf_rd_data
//        ▼
//   rounding_shifter  ◄── quant_shift_amt
//        │  quant_data
//        ├───────────────────────────────────────┐
//        ▼                                       │ (bypass)
//      relu                                      │
//        │  relu_data                            │
//        ▼                                       ▼
//   post_proc_mux  ◄── relu_en  ──  selects activated / linear
//        │
//        │  post_proc_data
//        ├──────────────────────────────► output_memory.wr_data  (DIRECT)
//        │
//                output_memory.wr_addr / wr_en  ◄── unified_controller
//
// ── Feedback path ─────────────────────────────────────────────────────────
//   omem_fb_en = 0 (normal)
//     unified_controller.imem_rd_addr → fib_memory.rd_addr
//     fib_memory.rd_data              → unified_controller.imem_rd_data
//
//   omem_fb_en = 1 (feedback — layer chaining)
//     unified_controller.imem_rd_addr → output_memory.fb_rd_addr
//     output_memory.fb_rd_data        → unified_controller.imem_rd_data
//     (1-cycle delayed flag aligns data steering with registered memory output)
// =============================================================================

module full_system_top (
    input  logic clk,
    input  logic rst_n,

    // ── Mode and control ───────────────────────────────────────────────────
    input  logic mode,      // 0 = Conv, 1 = MLP
    input  logic start,
    output logic done,

    // ── Post-processing controls ──────────────────────────────────────────
    input  logic signed [7:0] quant_shift_amt,
    input  logic               relu_en,

    // ── Feedback control ──────────────────────────────────────────────────
    input  logic               omem_fb_en,   // 1 → route imem to omem.fb port

    // ── CPU/DMA: weight_memory write ──────────────────────────────────────
    input  logic [14:0] cpu_wmem_wr_addr,
    input  logic [31:0] cpu_wmem_wr_data,
    input  logic        cpu_wmem_wr_en,

    // ── CPU/DMA: fib_memory write ─────────────────────────────────────────
    input  logic [16:0] cpu_fib_wr_addr,
    input  logic [31:0] cpu_fib_wr_data,
    input  logic        cpu_fib_wr_en,

    // ── CPU/DMA: output_memory read ───────────────────────────────────────
    input  logic [18:0] cpu_omem_rd_addr,
    input  logic        cpu_omem_rd_en,
    output logic [31:0] cpu_omem_rd_data
);

// =============================================================================
// Localparams (mirror unified_controller parameter values)
// =============================================================================
localparam int WAW = 15;
localparam int FAW = 17;
localparam int OAW = 19;

// =============================================================================
// Unified controller output wires
// =============================================================================

// ── Memories ──────────────────────────────────────────────────────────────
logic [WAW-1:0] ctrl_wmem_rd_addr;
logic           ctrl_wmem_rd_en;
logic [31:0]    ctrl_wmem_rd_data;

logic [OAW-1:0] ctrl_imem_rd_addr;   // used for both fib and omem.fb paths
logic           ctrl_imem_rd_en;
logic [31:0]    ctrl_imem_rd_data;   // mux output, wired back to controller

logic [OAW-1:0] ctrl_omem_wr_addr;
logic           ctrl_omem_wr_en;

// ── Weight buffer ──────────────────────────────────────────────────────────
logic           ctrl_wbuf_load_en;
logic [3:0]     ctrl_wbuf_load_pe_idx;
logic [6:0]     ctrl_wbuf_load_k_word;
logic [31:0]    ctrl_wbuf_load_data;
logic           ctrl_wbuf_bias_load_en;
logic [31:0]    ctrl_wbuf_bias_load_data;
logic           ctrl_wbuf_swap;

// ── Input buffer ───────────────────────────────────────────────────────────
logic           ctrl_ibuf_load_en;
logic [3:0]     ctrl_ibuf_load_pe_idx;
logic [2:0]     ctrl_ibuf_load_win_idx;
logic [2:0]     ctrl_ibuf_load_row;
logic [6:0]     ctrl_ibuf_load_k_word;
logic [31:0]    ctrl_ibuf_load_data;
logic           ctrl_ibuf_swap;
logic           ctrl_ibuf_l1_capture_en;
logic [8:0]     ctrl_ibuf_l1_col_wr;

// ── MMU ────────────────────────────────────────────────────────────────────
logic           ctrl_mmu_valid_in;
logic [2:0]     ctrl_mmu_op_code;
logic [1:0]     ctrl_mmu_stage;
logic [2:0]     ctrl_mmu_sub_cycle;

// ── Output buffer ──────────────────────────────────────────────────────────
logic           ctrl_obuf_capture_en;
logic [2:0]     ctrl_obuf_rd_idx;

// =============================================================================
// Physical memory wires
// =============================================================================
logic [31:0]    wmem_rd_data_phys;

logic [FAW-1:0] fib_rd_addr;
logic           fib_rd_en;
logic [31:0]    fib_rd_data;

logic [OAW-1:0] omem_fb_rd_addr;
logic           omem_fb_rd_en;
logic [31:0]    omem_fb_rd_data;

// =============================================================================
// Feedback path mux
// (replaces the input-source section of the former DSU)
// =============================================================================
//
// 1-cycle delayed flag aligns rd_data steering with the registered memory output.
logic omem_fb_en_d;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) omem_fb_en_d <= 1'b0;
    else        omem_fb_en_d <= omem_fb_en;
end

// Route controller's imem request to the selected physical memory
always_comb begin
    fib_rd_addr    = '0; fib_rd_en    = 1'b0;
    omem_fb_rd_addr= '0; omem_fb_rd_en= 1'b0;
    if (!omem_fb_en) begin
        fib_rd_addr = ctrl_imem_rd_addr[FAW-1:0];
        fib_rd_en   = ctrl_imem_rd_en;
    end else begin
        omem_fb_rd_addr = ctrl_imem_rd_addr;
        omem_fb_rd_en   = ctrl_imem_rd_en;
    end
end

// Steer the registered read-data back to the controller
assign ctrl_imem_rd_data = omem_fb_en_d ? omem_fb_rd_data : fib_rd_data;

// =============================================================================
// Unified buffers → MMU bus wires
// =============================================================================
logic [7:0]  ubuf_w_out   [0:11][0:3];
logic [31:0] ubuf_bias_out;
logic [7:0]  ubuf_in_out  [0:11][0:6][0:3];

// =============================================================================
// MMU wires
// =============================================================================
logic        mmu_valid_out;
logic [7:0]  mmu_in_bus   [0:11][0:6][0:3];
logic [7:0]  mmu_w_bus    [0:11][0:3];
logic [31:0] mmu_bias_bus [0:11];
logic [31:0] mmu_out      [0:6];

// =============================================================================
// Output buffer / post-processing wires
// =============================================================================
logic [31:0]        obuf_rd_data;
logic signed [31:0] quant_data;
logic signed [31:0] relu_data;
logic signed [31:0] post_proc_data;

// =============================================================================
// MMU bus wiring  (weight buffer + input buffer → MMU)
// =============================================================================
always_comb begin
    for (int p = 0; p < 12; p++) begin
        for (int t = 0; t < 4; t++)
            mmu_w_bus[p][t] = ubuf_w_out[p][t];
        // Only PE 0 receives the accumulated bias; ubuf outputs 0 in MLP mode
        mmu_bias_bus[p] = (p == 0) ? ubuf_bias_out : 32'd0;
        for (int w = 0; w < 7; w++)
            for (int t = 0; t < 4; t++)
                mmu_in_bus[p][w][t] = ubuf_in_out[p][w][t];
    end
end

// =============================================================================
// Instance: unified_controller
// =============================================================================
unified_controller #(
    .WAW     (WAW),
    .FAW     (FAW),
    .OAW     (OAW),
    .W2_BASE (9216)
) u_ctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .mode                 (mode),
    .start                (start),
    .done                 (done),

    .wmem_rd_addr         (ctrl_wmem_rd_addr),
    .wmem_rd_en           (ctrl_wmem_rd_en),
    .wmem_rd_data         (wmem_rd_data_phys),

    .imem_rd_addr         (ctrl_imem_rd_addr),
    .imem_rd_en           (ctrl_imem_rd_en),
    .imem_rd_data         (ctrl_imem_rd_data),

    .omem_wr_addr         (ctrl_omem_wr_addr),
    .omem_wr_en           (ctrl_omem_wr_en),

    .wbuf_load_en         (ctrl_wbuf_load_en),
    .wbuf_load_pe_idx     (ctrl_wbuf_load_pe_idx),
    .wbuf_load_k_word     (ctrl_wbuf_load_k_word),
    .wbuf_load_data       (ctrl_wbuf_load_data),
    .wbuf_bias_load_en    (ctrl_wbuf_bias_load_en),
    .wbuf_bias_load_data  (ctrl_wbuf_bias_load_data),
    .wbuf_swap            (ctrl_wbuf_swap),

    .ibuf_load_en         (ctrl_ibuf_load_en),
    .ibuf_load_pe_idx     (ctrl_ibuf_load_pe_idx),
    .ibuf_load_win_idx    (ctrl_ibuf_load_win_idx),
    .ibuf_load_row        (ctrl_ibuf_load_row),
    .ibuf_load_k_word     (ctrl_ibuf_load_k_word),
    .ibuf_load_data       (ctrl_ibuf_load_data),
    .ibuf_swap            (ctrl_ibuf_swap),
    .ibuf_l1_capture_en   (ctrl_ibuf_l1_capture_en),
    .ibuf_l1_col_wr       (ctrl_ibuf_l1_col_wr),

    .mmu_valid_in         (ctrl_mmu_valid_in),
    .mmu_op_code          (ctrl_mmu_op_code),
    .mmu_stage            (ctrl_mmu_stage),
    .mmu_sub_cycle        (ctrl_mmu_sub_cycle),

    .obuf_capture_en      (ctrl_obuf_capture_en),
    .obuf_rd_idx          (ctrl_obuf_rd_idx)
);

// =============================================================================
// Instance: weight_memory
// =============================================================================
weight_memory u_wmem (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_addr (cpu_wmem_wr_addr),
    .wr_data (cpu_wmem_wr_data),
    .wr_en   (cpu_wmem_wr_en),
    .rd_addr (ctrl_wmem_rd_addr),
    .rd_en   (ctrl_wmem_rd_en),
    .rd_data (wmem_rd_data_phys)
);

// =============================================================================
// Instance: fib_memory
// =============================================================================
fib_memory u_fib (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_addr (cpu_fib_wr_addr),
    .wr_data (cpu_fib_wr_data),
    .wr_en   (cpu_fib_wr_en),
    .rd_addr (fib_rd_addr),
    .rd_en   (fib_rd_en),
    .rd_data (fib_rd_data)
);

// =============================================================================
// Instance: output_memory
//   wr_data  ← post_proc_data         (DIRECT from post-processing mux)
//   wr_addr  ← ctrl_omem_wr_addr      (from unified_controller)
//   wr_en    ← ctrl_omem_wr_en        (from unified_controller)
//   fb_rd_*  ↔ feedback mux above     (layer-chaining path)
// =============================================================================
output_memory u_omem (
    .clk         (clk),
    .rst_n       (rst_n),
    .wr_addr     (ctrl_omem_wr_addr),
    .wr_data     (post_proc_data),       // direct wire — not through controller
    .wr_en       (ctrl_omem_wr_en),
    .cpu_rd_addr (cpu_omem_rd_addr),
    .cpu_rd_en   (cpu_omem_rd_en),
    .cpu_rd_data (cpu_omem_rd_data),
    .fb_rd_addr  (omem_fb_rd_addr),
    .fb_rd_en    (omem_fb_rd_en),
    .fb_rd_data  (omem_fb_rd_data)
);

// =============================================================================
// Instance: unified_weight_buf
//   conv_load_en / mlp_load_en gated by mode so only the active path writes.
// =============================================================================
unified_weight_buf u_wbuf (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),
    .swap                (ctrl_wbuf_swap),

    // Conv weight load  (gated: active only in Conv mode)
    .conv_load_en        (~mode & ctrl_wbuf_load_en),
    .conv_load_pe_idx    (ctrl_wbuf_load_pe_idx),
    .conv_load_data      (ctrl_wbuf_load_data),
    .conv_bias_load_en   (~mode & ctrl_wbuf_bias_load_en),
    .conv_bias_load_data (ctrl_wbuf_bias_load_data),

    // MLP weight load   (gated: active only in MLP mode)
    .mlp_load_en         (mode & ctrl_wbuf_load_en),
    .mlp_load_k_word     (ctrl_wbuf_load_k_word),
    .mlp_load_data       (ctrl_wbuf_load_data),

    .sub_cycle           (ctrl_mmu_sub_cycle),
    .w_out               (ubuf_w_out),
    .bias_out            (ubuf_bias_out)
);

// =============================================================================
// Instance: unified_input_buf
//   Load enables gated by mode for the same reason as weight buf.
//   mlp_l1_out is wired to live mmu_out (Y captured directly from MMU).
// =============================================================================
unified_input_buf u_ibuf (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),
    .swap                (ctrl_ibuf_swap),

    // Conv image load  (gated)
    .conv_load_en        (~mode & ctrl_ibuf_load_en),
    .conv_load_pe_idx    (ctrl_ibuf_load_pe_idx),
    .conv_load_win_idx   (ctrl_ibuf_load_win_idx),
    .conv_load_data      (ctrl_ibuf_load_data),

    // MLP X load       (gated)
    .mlp_load_en         (mode & ctrl_ibuf_load_en),
    .mlp_load_row        (ctrl_ibuf_load_row),
    .mlp_load_k_word     (ctrl_ibuf_load_k_word),
    .mlp_load_data       (ctrl_ibuf_load_data),

    // MLP L1 Y capture (shadow bank, not gated — only asserted in MLP states)
    .mlp_capture_en      (ctrl_ibuf_l1_capture_en),
    .mlp_col_wr          (ctrl_ibuf_l1_col_wr),
    .mlp_l1_out          (mmu_out),                 // live MMU output

    .sub_cycle           (ctrl_mmu_sub_cycle),
    .data_out            (ubuf_in_out)
);

// =============================================================================
// Instance: mmu_top
// =============================================================================
mmu_top u_mmu (
    .clk       (clk),
    .rst_n     (rst_n),
    .valid_in  (ctrl_mmu_valid_in),
    .op_code   (ctrl_mmu_op_code),
    .stage     (ctrl_mmu_stage),
    .valid_out (mmu_valid_out),
    .mmu_in    (mmu_in_bus),
    .mmu_w     (mmu_w_bus),
    .mmu_bias  (mmu_bias_bus),
    .mmu_out   (mmu_out)
);

// =============================================================================
// Instance: output_buffer
//   Captures mmu_out when ctrl_obuf_capture_en is asserted.
//   Presents one word per cycle to the post-processing pipeline during WB.
// =============================================================================
output_buffer u_obuf (
    .clk        (clk),
    .rst_n      (rst_n),
    .capture_en (ctrl_obuf_capture_en),
    .mmu_out    (mmu_out),
    .rd_idx     (ctrl_obuf_rd_idx),
    .rd_data    (obuf_rd_data)
);

// =============================================================================
// Post-processing pipeline  (purely combinatorial — zero added latency)
//
//   obuf_rd_data
//       │
//       ▼
//   rounding_shifter  ◄── quant_shift_amt
//       │  quant_data
//       ├───────────────────────────────────────┐
//       ▼                                       │ (bypass when relu_en=0)
//     relu                                      │
//       │  relu_data                            │
//       ▼                                       ▼
//   post_proc_mux  ◄── relu_en  ──  data_out = post_proc_data
//       │
//       └──► output_memory.wr_data  (direct wire)
// =============================================================================

rounding_shifter #(.W_INPUT(32), .W_SHIFT(8)) u_quantizer (
    .in_value  ($signed(obuf_rd_data)),
    .shift_amt (quant_shift_amt),
    .out_value (quant_data)
);

relu #(.W(32)) u_relu (
    .in_value  (quant_data),
    .out_value (relu_data)
);

post_proc_mux #(.W(32)) u_mux (
    .relu_in  (relu_data),
    .quant_in (quant_data),
    .relu_en  (relu_en),
    .data_out (post_proc_data)     // wired directly to output_memory.wr_data
);

endmodule
