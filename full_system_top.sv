// =============================================================================
// full_system_top.sv  (rev 4)
//
// ── Corrected post-processing data flow ──────────────────────────────────
//
//   output_buffer
//        │  obuf_rd_data
//        ▼
//   rounding_shifter  ← quant_shift_amt
//        │  quant_data
//        ├─────────────────────────────┐
//        ▼                             │ (bypass)
//      relu                            │
//        │  relu_data                  │
//        ▼                             ▼
//   post_proc_mux  ◄── relu_en ──  selects
//        │
//        │  post_proc_data
//        │
//        ├──────────────────────────────────────────────────────────────────►
//        │                              output_memory.wr_data  (DIRECT wire)
//        │                              output_memory.wr_addr  ◄── DSU
//        │                              output_memory.wr_en    ◄── DSU
//        │
//        │                              output_memory.fb_rd_data (feedback)
//        │                                     │
//        │                                    DSU.omem_fb_rd_data
//        │                                     │
//        │                              conv_imem_rd_data / fc_xmem_rd_data
//        │                                     │
//        │                              input buffers ──► MMU
//
// KEY POINT: post_proc_data is wired directly to output_memory.wr_data.
//            The DSU is no longer in the write-data path at all.
//            The DSU only drives wr_addr and wr_en for output_memory.
//
// ── Control ports ─────────────────────────────────────────────────────────
//   quant_shift_amt : signed 8-bit shift for rounding_shifter
//   relu_en         : 1 = ReLU activated output,  0 = quantiser bypass
//   omem_fb_en      : 1 = feedback active (output_memory → DSU input path)
//                     0 = normal (fib_memory → DSU input path)
// =============================================================================

module full_system_top (
    input  logic clk,
    input  logic rst_n,

    // ── Mode and control ───────────────────────────────────────────────────
    input  logic mode,
    input  logic start,
    output logic done,

    // ── Post-processing controls ──────────────────────────────────────────
    input  logic signed [7:0] quant_shift_amt,
    input  logic               relu_en,

    // ── Feedback control ──────────────────────────────────────────────────
    input  logic               omem_fb_en,

    // ── CPU/DMA: weight_memory write port ─────────────────────────────────
    input  logic [14:0] cpu_wmem_wr_addr,
    input  logic [31:0] cpu_wmem_wr_data,
    input  logic        cpu_wmem_wr_en,

    // ── CPU/DMA: fib_memory write port ────────────────────────────────────
    input  logic [16:0] cpu_fib_wr_addr,
    input  logic [31:0] cpu_fib_wr_data,
    input  logic        cpu_fib_wr_en,

    // ── CPU/DMA: output_memory read port ──────────────────────────────────
    input  logic [18:0] cpu_omem_rd_addr,
    input  logic        cpu_omem_rd_en,
    output logic [31:0] cpu_omem_rd_data
);

// =============================================================================
// Internal wires — conv_controller
// =============================================================================
logic        conv_done;
logic [31:0] conv_wmem_addr,  conv_imem_addr,  conv_omem_addr;
logic        conv_wmem_rd_en, conv_imem_rd_en, conv_omem_wr_en;
logic [31:0] conv_wmem_rd_data, conv_imem_rd_data;

logic        conv_wbuf_load_en,   conv_wbuf_bias_load_en, conv_wbuf_swap;
logic [3:0]  conv_wbuf_load_pe_idx;
logic [31:0] conv_wbuf_load_data, conv_wbuf_bias_load_data;

logic        conv_ibuf_load_en, conv_ibuf_swap;
logic [3:0]  conv_ibuf_load_pe_idx;
logic [2:0]  conv_ibuf_load_win_idx;
logic [31:0] conv_ibuf_load_data;

logic        conv_mmu_valid_in, conv_mmu_capture_en;
logic [2:0]  conv_obuf_rd_idx;

// =============================================================================
// Internal wires — fc_controller
// =============================================================================
logic        fc_done;
logic [31:0] fc_w1mem_addr,  fc_w2mem_addr,  fc_xmem_addr,  fc_omem_addr;
logic        fc_w1mem_rd_en, fc_w2mem_rd_en, fc_xmem_rd_en, fc_omem_wr_en;
logic [31:0] fc_w1mem_rd_data, fc_w2mem_rd_data, fc_xmem_rd_data;

logic        fc_wbuf_load_en, fc_wbuf_swap;
logic [6:0]  fc_wbuf_load_k_word;
logic [31:0] fc_wbuf_load_data;

logic        fc_ibuf_load_en, fc_ibuf_swap;
logic [2:0]  fc_ibuf_load_row;
logic [6:0]  fc_ibuf_load_k_word;
logic [31:0] fc_ibuf_load_data;

logic        fc_ibuf_l1_capture_en;
logic [8:0]  fc_ibuf_l1_col_wr;

logic        fc_mmu_valid_in;
logic [2:0]  fc_mmu_op_code, fc_mmu_sub_cycle;
logic [1:0]  fc_mmu_stage;

logic        fc_l2_capture_en;
logic [2:0]  fc_obuf_rd_idx;

// =============================================================================
// Internal wires — DSU ↔ memories
// =============================================================================
logic [14:0] dsu_wmem_rd_addr;
logic        dsu_wmem_rd_en;
logic [31:0] dsu_wmem_rd_data;

logic [16:0] dsu_fib_rd_addr;
logic        dsu_fib_rd_en;
logic [31:0] dsu_fib_rd_data;

// DSU drives only address + enable for output_memory write (no data)
logic [18:0] dsu_omem_wr_addr;
logic        dsu_omem_wr_en;

// DSU drives output_memory feedback read port
logic [18:0] dsu_omem_fb_rd_addr;
logic        dsu_omem_fb_rd_en;
logic [31:0] dsu_omem_fb_rd_data;

// =============================================================================
// Internal wires — unified buffers → MMU
// =============================================================================
logic [7:0]  ubuf_w_out  [0:11][0:3];
logic [31:0] ubuf_bias_out;
logic [7:0]  ubuf_in_out [0:11][0:6][0:3];

// =============================================================================
// Internal wires — MMU
// =============================================================================
logic        mmu_valid_in;
logic [2:0]  mmu_op_code;
logic [1:0]  mmu_stage;
logic        mmu_valid_out;
logic [7:0]  mmu_in_bus  [0:11][0:6][0:3];
logic [7:0]  mmu_w_bus   [0:11][0:3];
logic [31:0] mmu_bias_bus[0:11];
logic [31:0] mmu_out     [0:6];

// =============================================================================
// Internal wires — output_buffer
// =============================================================================
logic        obuf_capture_en;
logic [2:0]  obuf_rd_idx;
logic [31:0] obuf_rd_data;

// =============================================================================
// Internal wires — post-processing chain
// =============================================================================
logic signed [31:0] quant_data;       // after rounding_shifter
logic signed [31:0] relu_data;        // after relu
logic signed [31:0] post_proc_data;   // after mux — final value to output_memory

// =============================================================================
// Mode-gated done
// =============================================================================
assign done = (mode == 1'b0) ? conv_done : fc_done;

// =============================================================================
// MMU control mux
// =============================================================================
always_comb begin
    if (mode == 1'b0) begin
        mmu_valid_in = conv_mmu_valid_in;
        mmu_op_code  = 3'd0;
        mmu_stage    = 2'd0;
    end else begin
        mmu_valid_in = fc_mmu_valid_in;
        mmu_op_code  = fc_mmu_op_code;
        mmu_stage    = fc_mmu_stage;
    end
end

// =============================================================================
// MMU bus wiring
// =============================================================================
always_comb begin
    for (int p = 0; p < 12; p++) begin
        for (int t = 0; t < 4; t++)
            mmu_w_bus[p][t] = ubuf_w_out[p][t];
        mmu_bias_bus[p] = (mode == 1'b0 && p == 0) ? ubuf_bias_out : 32'd0;
        for (int w = 0; w < 7; w++)
            for (int t = 0; t < 4; t++)
                mmu_in_bus[p][w][t] = ubuf_in_out[p][w][t];
    end
end

// =============================================================================
// Output buffer capture & readout mux
// =============================================================================
assign obuf_capture_en = (mode == 1'b0) ? conv_mmu_capture_en : fc_l2_capture_en;
assign obuf_rd_idx     = (mode == 1'b0) ? conv_obuf_rd_idx    : fc_obuf_rd_idx;

// =============================================================================
// Instance: conv_controller
// =============================================================================
conv_controller u_conv_ctrl (
    .clk                 (clk),
    .rst_n               (rst_n),
    .start               (start & ~mode),
    .done                (conv_done),
    .wmem_addr           (conv_wmem_addr),
    .wmem_rd_en          (conv_wmem_rd_en),
    .wmem_rd_data        (conv_wmem_rd_data),
    .imem_addr           (conv_imem_addr),
    .imem_rd_en          (conv_imem_rd_en),
    .imem_rd_data        (conv_imem_rd_data),
    .omem_addr           (conv_omem_addr),
    .omem_wr_en          (conv_omem_wr_en),
    .wbuf_load_en        (conv_wbuf_load_en),
    .wbuf_load_pe_idx    (conv_wbuf_load_pe_idx),
    .wbuf_load_data      (conv_wbuf_load_data),
    .wbuf_bias_load_en   (conv_wbuf_bias_load_en),
    .wbuf_bias_load_data (conv_wbuf_bias_load_data),
    .wbuf_swap           (conv_wbuf_swap),
    .ibuf_load_en        (conv_ibuf_load_en),
    .ibuf_load_pe_idx    (conv_ibuf_load_pe_idx),
    .ibuf_load_win_idx   (conv_ibuf_load_win_idx),
    .ibuf_load_data      (conv_ibuf_load_data),
    .ibuf_swap           (conv_ibuf_swap),
    .mmu_valid_in        (conv_mmu_valid_in),
    .mmu_capture_en      (conv_mmu_capture_en),
    .obuf_rd_idx         (conv_obuf_rd_idx)
);

// =============================================================================
// Instance: fc_controller
// =============================================================================
fc_controller u_fc_ctrl (
    .clk                 (clk),
    .rst_n               (rst_n),
    .start               (start & mode),
    .done                (fc_done),
    .w1mem_addr          (fc_w1mem_addr),
    .w1mem_rd_en         (fc_w1mem_rd_en),
    .w1mem_rd_data       (fc_w1mem_rd_data),
    .w2mem_addr          (fc_w2mem_addr),
    .w2mem_rd_en         (fc_w2mem_rd_en),
    .w2mem_rd_data       (fc_w2mem_rd_data),
    .xmem_addr           (fc_xmem_addr),
    .xmem_rd_en          (fc_xmem_rd_en),
    .xmem_rd_data        (fc_xmem_rd_data),
    .omem_addr           (fc_omem_addr),
    .omem_wr_en          (fc_omem_wr_en),
    .wbuf_load_en        (fc_wbuf_load_en),
    .wbuf_load_k_word    (fc_wbuf_load_k_word),
    .wbuf_load_data      (fc_wbuf_load_data),
    .wbuf_swap           (fc_wbuf_swap),
    .ibuf_load_en        (fc_ibuf_load_en),
    .ibuf_load_row       (fc_ibuf_load_row),
    .ibuf_load_k_word    (fc_ibuf_load_k_word),
    .ibuf_load_data      (fc_ibuf_load_data),
    .ibuf_swap           (fc_ibuf_swap),
    .ibuf_l1_capture_en  (fc_ibuf_l1_capture_en),
    .ibuf_l1_col_wr      (fc_ibuf_l1_col_wr),
    .mmu_valid_in        (fc_mmu_valid_in),
    .mmu_op_code         (fc_mmu_op_code),
    .mmu_stage           (fc_mmu_stage),
    .mmu_sub_cycle       (fc_mmu_sub_cycle),
    .l2_capture_en       (fc_l2_capture_en),
    .obuf_rd_idx         (fc_obuf_rd_idx)
);

// =============================================================================
// Instance: dsu
// NOTE: obuf_rd_data / omem_wr_data ports no longer exist on the DSU.
//       The DSU only drives omem_wr_addr and omem_wr_en.
// =============================================================================
dsu u_dsu (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),
    .omem_fb_en          (omem_fb_en),

    .conv_wmem_addr      (conv_wmem_addr),
    .conv_wmem_rd_en     (conv_wmem_rd_en),
    .conv_wmem_rd_data   (conv_wmem_rd_data),
    .conv_imem_addr      (conv_imem_addr),
    .conv_imem_rd_en     (conv_imem_rd_en),
    .conv_imem_rd_data   (conv_imem_rd_data),
    .conv_omem_addr      (conv_omem_addr),
    .conv_omem_wr_en     (conv_omem_wr_en),

    .fc_w1mem_addr       (fc_w1mem_addr),
    .fc_w1mem_rd_en      (fc_w1mem_rd_en),
    .fc_w1mem_rd_data    (fc_w1mem_rd_data),
    .fc_w2mem_addr       (fc_w2mem_addr),
    .fc_w2mem_rd_en      (fc_w2mem_rd_en),
    .fc_w2mem_rd_data    (fc_w2mem_rd_data),
    .fc_xmem_addr        (fc_xmem_addr),
    .fc_xmem_rd_en       (fc_xmem_rd_en),
    .fc_xmem_rd_data     (fc_xmem_rd_data),
    .fc_omem_addr        (fc_omem_addr),
    .fc_omem_wr_en       (fc_omem_wr_en),

    .wmem_rd_addr        (dsu_wmem_rd_addr),
    .wmem_rd_en          (dsu_wmem_rd_en),
    .wmem_rd_data        (dsu_wmem_rd_data),

    .fib_rd_addr         (dsu_fib_rd_addr),
    .fib_rd_en           (dsu_fib_rd_en),
    .fib_rd_data         (dsu_fib_rd_data),

    // Write address + enable only — no data port
    .omem_wr_addr        (dsu_omem_wr_addr),
    .omem_wr_en          (dsu_omem_wr_en),

    // Feedback read: DSU requests stored results from output_memory
    .omem_fb_rd_addr     (dsu_omem_fb_rd_addr),
    .omem_fb_rd_en       (dsu_omem_fb_rd_en),
    .omem_fb_rd_data     (dsu_omem_fb_rd_data)
);

// =============================================================================
// Instance: weight_memory
// =============================================================================
weight_memory u_wmem (
    .clk     (clk),   .rst_n   (rst_n),
    .wr_addr (cpu_wmem_wr_addr),
    .wr_data (cpu_wmem_wr_data),
    .wr_en   (cpu_wmem_wr_en),
    .rd_addr (dsu_wmem_rd_addr),
    .rd_en   (dsu_wmem_rd_en),
    .rd_data (dsu_wmem_rd_data)
);

// =============================================================================
// Instance: fib_memory
// =============================================================================
fib_memory u_fib (
    .clk     (clk),   .rst_n   (rst_n),
    .wr_addr (cpu_fib_wr_addr),
    .wr_data (cpu_fib_wr_data),
    .wr_en   (cpu_fib_wr_en),
    .rd_addr (dsu_fib_rd_addr),
    .rd_en   (dsu_fib_rd_en),
    .rd_data (dsu_fib_rd_data)
);

// =============================================================================
// Instance: output_memory
// ─────────────────────────────────────────────────────────────────────────────
//   wr_data  ← post_proc_data    DIRECT wire from the post-processing mux
//   wr_addr  ← dsu_omem_wr_addr  from DSU (controller FSM address)
//   wr_en    ← dsu_omem_wr_en    from DSU (WRITEBACK state gate)
//   fb_rd_*  ↔ dsu_omem_fb_*     feedback loop back to DSU input path
// =============================================================================
output_memory u_omem (
    .clk         (clk),
    .rst_n       (rst_n),

    // Write port — data comes DIRECTLY from the post-processing mux output
    .wr_addr     (dsu_omem_wr_addr),
    .wr_data     (post_proc_data),        // ← direct wire, NOT through DSU
    .wr_en       (dsu_omem_wr_en),

    // CPU readback
    .cpu_rd_addr (cpu_omem_rd_addr),
    .cpu_rd_en   (cpu_omem_rd_en),
    .cpu_rd_data (cpu_omem_rd_data),

    // Feedback read — output_memory sends stored results back to DSU
    .fb_rd_addr  (dsu_omem_fb_rd_addr),
    .fb_rd_en    (dsu_omem_fb_rd_en),
    .fb_rd_data  (dsu_omem_fb_rd_data)   // ← flows into DSU input mux
);

// =============================================================================
// Instance: unified_weight_buf
// =============================================================================
unified_weight_buf u_wbuf (
    .clk                 (clk),   .rst_n               (rst_n),
    .mode                (mode),
    .swap                ((mode == 1'b0) ? conv_wbuf_swap : fc_wbuf_swap),
    .conv_load_en        (conv_wbuf_load_en),
    .conv_load_pe_idx    (conv_wbuf_load_pe_idx),
    .conv_load_data      (conv_wbuf_load_data),
    .conv_bias_load_en   (conv_wbuf_bias_load_en),
    .conv_bias_load_data (conv_wbuf_bias_load_data),
    .mlp_load_en         (fc_wbuf_load_en),
    .mlp_load_k_word     (fc_wbuf_load_k_word),
    .mlp_load_data       (fc_wbuf_load_data),
    .sub_cycle           (fc_mmu_sub_cycle),
    .w_out               (ubuf_w_out),
    .bias_out            (ubuf_bias_out)
);

// =============================================================================
// Instance: unified_input_buf
// =============================================================================
unified_input_buf u_ibuf (
    .clk                 (clk),   .rst_n               (rst_n),
    .mode                (mode),
    .swap                ((mode == 1'b0) ? conv_ibuf_swap : fc_ibuf_swap),
    .conv_load_en        (conv_ibuf_load_en),
    .conv_load_pe_idx    (conv_ibuf_load_pe_idx),
    .conv_load_win_idx   (conv_ibuf_load_win_idx),
    .conv_load_data      (conv_ibuf_load_data),
    .mlp_load_en         (fc_ibuf_load_en),
    .mlp_load_row        (fc_ibuf_load_row),
    .mlp_load_k_word     (fc_ibuf_load_k_word),
    .mlp_load_data       (fc_ibuf_load_data),
    .mlp_capture_en      (fc_ibuf_l1_capture_en),
    .mlp_col_wr          (fc_ibuf_l1_col_wr),
    .mlp_l1_out          (mmu_out),
    .sub_cycle           (fc_mmu_sub_cycle),
    .data_out            (ubuf_in_out)
);

// =============================================================================
// Instance: mmu_top
// =============================================================================
mmu_top u_mmu (
    .clk       (clk),   .rst_n     (rst_n),
    .valid_in  (mmu_valid_in),
    .op_code   (mmu_op_code),
    .stage     (mmu_stage),
    .valid_out (mmu_valid_out),
    .mmu_in    (mmu_in_bus),
    .mmu_w     (mmu_w_bus),
    .mmu_bias  (mmu_bias_bus),
    .mmu_out   (mmu_out)
);

// =============================================================================
// Instance: output_buffer
// =============================================================================
output_buffer u_obuf (
    .clk        (clk),
    .rst_n      (rst_n),
    .capture_en (obuf_capture_en),
    .mmu_out    (mmu_out),
    .rd_idx     (obuf_rd_idx),
    .rd_data    (obuf_rd_data)
);

// =============================================================================
// Post-processing pipeline  (purely combinatorial — zero added latency)
//
//   obuf_rd_data ──► rounding_shifter ──► relu ──► post_proc_mux
//                                                         │
//                                                  post_proc_data
//                                                         │
//                                              output_memory.wr_data  (direct)
// =============================================================================

// Stage 1 — quantiser
rounding_shifter #(.W_INPUT(32), .W_SHIFT(8)) u_quantizer (
    .in_value  ($signed(obuf_rd_data)),
    .shift_amt (quant_shift_amt),
    .out_value (quant_data)
);

// Stage 2 — ReLU
relu #(.W(32)) u_relu (
    .in_value  (quant_data),
    .out_value (relu_data)
);

// Stage 3 — activation mux
// relu_en=1 → relu output (activated);  relu_en=0 → quantiser bypass (linear)
post_proc_mux #(.W(32)) u_mux (
    .relu_in   (relu_data),
    .quant_in  (quant_data),
    .relu_en   (relu_en),
    .data_out  (post_proc_data)    // wired directly to output_memory.wr_data
);

endmodule