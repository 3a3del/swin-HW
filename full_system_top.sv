// =============================================================================
// full_system_top.sv  —  Complete integrated top-level
//
// ── New blocks added vs. previous version ────────────────────────────────
//   weight_memory : large SRAM holding ALL weight data (Conv + MLP)
//   fib_memory    : large SRAM holding ALL input data  (Conv image + MLP X)
//   output_memory : large SRAM holding computation results
//   dsu           : Data Selection Unit — routes memory ↔ controller traffic
//
// ── Memory address map (time-shared, one mode active at a time) ───────────
//
//   weight_memory  (18 432 × 32-bit words):
//     Conv  mode:  w[0..1247]     96k × 13w  (12 PE-weights + bias per kernel)
//     MLP   mode:  W1[0..9215]    384c × 24w (col-major W1)
//                  W2[9216..18431] 96c × 96w  (col-major W2, DSU adds offset)
//
//   fib_memory  (75 264 × 32-bit words):
//     Conv  mode:  image[0..37631]  CHW 224×224×3 / 4px-per-word
//     MLP   mode:  X[0..75263]      3136r × 24w (row-major)
//
//   output_memory  (301 056 × 32-bit words):
//     Conv  mode:  out[0..301055]   56×56×96
//     MLP   mode:  Z  [0..301055]   3136×96
//
// ── Data flow ─────────────────────────────────────────────────────────────
//
//   CONV:
//     weight_memory ──(DSU)──► conv_ctrl ──► wbuf_load ──► unified_weight_buf
//     fib_memory    ──(DSU)──► conv_ctrl ──► ibuf_load ──► unified_input_buf
//     unified_weight_buf / unified_input_buf ──► mmu_top ──► output_buffer
//     output_buffer ──► (DSU) ──► output_memory
//
//   MLP:
//     weight_memory ──(DSU)──► fc_ctrl   ──► wbuf_load ──► unified_weight_buf
//     fib_memory    ──(DSU)──► fc_ctrl   ──► ibuf_load ──► unified_input_buf
//     L1 output ──► ibuf shadow (Y accumulation, no DSU)
//     L2 output ──► output_buffer ──► (DSU) ──► output_memory
//
// ── External interfaces ───────────────────────────────────────────────────
//   CPU/DMA write ports for loading memories before engine start.
//   CPU/DMA read port  on output_memory for reading results after done.
// =============================================================================

module full_system_top (
    input  logic clk,
    input  logic rst_n,

    // ── Mode and control ───────────────────────────────────────────────────
    input  logic mode,          // 0 = Conv, 1 = MLP
    input  logic start,         // 1-cycle pulse to begin
    output logic done,

    // ── CPU/DMA: weight_memory write port ────────────────────────────────
    input  logic [14:0] cpu_wmem_wr_addr,
    input  logic [31:0] cpu_wmem_wr_data,
    input  logic        cpu_wmem_wr_en,

    // ── CPU/DMA: fib_memory write port ────────────────────────────────────
    input  logic [16:0] cpu_fib_wr_addr,
    input  logic [31:0] cpu_fib_wr_data,
    input  logic        cpu_fib_wr_en,

    // ── CPU/DMA: output_memory read port (result readback) ────────────────
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

logic [18:0] dsu_omem_wr_addr;
logic [31:0] dsu_omem_wr_data;
logic        dsu_omem_wr_en;

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
// Internal wires — output buffer
// =============================================================================
logic        obuf_capture_en;
logic [2:0]  obuf_rd_idx;
logic [31:0] obuf_rd_data;

// =============================================================================
// Mode-gated start & done
// =============================================================================
assign done = (mode == 1'b0) ? conv_done : fc_done;

// =============================================================================
// MMU control mux  (mode-based)
// =============================================================================
always_comb begin
    if (mode == 1'b0) begin
        mmu_valid_in = conv_mmu_valid_in;
        mmu_op_code  = 3'd0;          // Conv streaming mode
        mmu_stage    = 2'd0;
    end else begin
        mmu_valid_in = fc_mmu_valid_in;
        mmu_op_code  = fc_mmu_op_code;
        mmu_stage    = fc_mmu_stage;
    end
end

// =============================================================================
// MMU bus wiring (from unified buffers)
// =============================================================================
always_comb begin
    for (int p = 0; p < 12; p++) begin
        for (int t = 0; t < 4; t++)
            mmu_w_bus[p][t] = ubuf_w_out[p][t];

        // Bias: only PE-0 carries bias in Conv mode; zero in MLP mode
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
    .wmem_rd_data        (conv_wmem_rd_data),   // from DSU

    .imem_addr           (conv_imem_addr),
    .imem_rd_en          (conv_imem_rd_en),
    .imem_rd_data        (conv_imem_rd_data),   // from DSU

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
    .w1mem_rd_data       (fc_w1mem_rd_data),    // from DSU

    .w2mem_addr          (fc_w2mem_addr),
    .w2mem_rd_en         (fc_w2mem_rd_en),
    .w2mem_rd_data       (fc_w2mem_rd_data),    // from DSU

    .xmem_addr           (fc_xmem_addr),
    .xmem_rd_en          (fc_xmem_rd_en),
    .xmem_rd_data        (fc_xmem_rd_data),     // from DSU

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
// =============================================================================
dsu u_dsu (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),

    // Conv controller ↔ DSU
    .conv_wmem_addr      (conv_wmem_addr),
    .conv_wmem_rd_en     (conv_wmem_rd_en),
    .conv_wmem_rd_data   (conv_wmem_rd_data),

    .conv_imem_addr      (conv_imem_addr),
    .conv_imem_rd_en     (conv_imem_rd_en),
    .conv_imem_rd_data   (conv_imem_rd_data),

    .conv_omem_addr      (conv_omem_addr),
    .conv_omem_wr_en     (conv_omem_wr_en),

    // FC controller ↔ DSU
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

    // Shared output buffer data bus
    .obuf_rd_data        (obuf_rd_data),

    // Physical memory ports
    .wmem_rd_addr        (dsu_wmem_rd_addr),
    .wmem_rd_en          (dsu_wmem_rd_en),
    .wmem_rd_data        (dsu_wmem_rd_data),

    .fib_rd_addr         (dsu_fib_rd_addr),
    .fib_rd_en           (dsu_fib_rd_en),
    .fib_rd_data         (dsu_fib_rd_data),

    .omem_wr_addr        (dsu_omem_wr_addr),
    .omem_wr_data        (dsu_omem_wr_data),
    .omem_wr_en          (dsu_omem_wr_en)
);

// =============================================================================
// Instance: weight_memory
// =============================================================================
weight_memory u_wmem (
    .clk       (clk),
    .rst_n     (rst_n),

    // CPU/DMA load
    .wr_addr   (cpu_wmem_wr_addr),
    .wr_data   (cpu_wmem_wr_data),
    .wr_en     (cpu_wmem_wr_en),

    // DSU read
    .rd_addr   (dsu_wmem_rd_addr),
    .rd_en     (dsu_wmem_rd_en),
    .rd_data   (dsu_wmem_rd_data)
);

// =============================================================================
// Instance: fib_memory
// =============================================================================
fib_memory u_fib (
    .clk       (clk),
    .rst_n     (rst_n),

    // CPU/DMA load
    .wr_addr   (cpu_fib_wr_addr),
    .wr_data   (cpu_fib_wr_data),
    .wr_en     (cpu_fib_wr_en),

    // DSU read
    .rd_addr   (dsu_fib_rd_addr),
    .rd_en     (dsu_fib_rd_en),
    .rd_data   (dsu_fib_rd_data)
);

// =============================================================================
// Instance: output_memory
// =============================================================================
output_memory u_omem (
    .clk       (clk),
    .rst_n     (rst_n),

    // DSU write (engine results)
    .wr_addr   (dsu_omem_wr_addr),
    .wr_data   (dsu_omem_wr_data),
    .wr_en     (dsu_omem_wr_en),

    // CPU/DMA read (result readback)
    .rd_addr   (cpu_omem_rd_addr),
    .rd_en     (cpu_omem_rd_en),
    .rd_data   (cpu_omem_rd_data)
);

// =============================================================================
// Instance: unified_weight_buf
// =============================================================================
unified_weight_buf u_wbuf (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),
    .swap                ((mode == 1'b0) ? conv_wbuf_swap : fc_wbuf_swap),

    // Conv load
    .conv_load_en        (conv_wbuf_load_en),
    .conv_load_pe_idx    (conv_wbuf_load_pe_idx),
    .conv_load_data      (conv_wbuf_load_data),
    .conv_bias_load_en   (conv_wbuf_bias_load_en),
    .conv_bias_load_data (conv_wbuf_bias_load_data),

    // MLP load
    .mlp_load_en         (fc_wbuf_load_en),
    .mlp_load_k_word     (fc_wbuf_load_k_word),
    .mlp_load_data       (fc_wbuf_load_data),

    // Sub-cycle for MLP read
    .sub_cycle           (fc_mmu_sub_cycle),

    // Read → MMU
    .w_out               (ubuf_w_out),
    .bias_out            (ubuf_bias_out)
);

// =============================================================================
// Instance: unified_input_buf
// =============================================================================
unified_input_buf u_ibuf (
    .clk                 (clk),
    .rst_n               (rst_n),
    .mode                (mode),
    .swap                ((mode == 1'b0) ? conv_ibuf_swap : fc_ibuf_swap),

    // Conv load (from FIB via conv_ctrl via DSU)
    .conv_load_en        (conv_ibuf_load_en),
    .conv_load_pe_idx    (conv_ibuf_load_pe_idx),
    .conv_load_win_idx   (conv_ibuf_load_win_idx),
    .conv_load_data      (conv_ibuf_load_data),

    // MLP X load (from FIB via fc_ctrl via DSU)
    .mlp_load_en         (fc_ibuf_load_en),
    .mlp_load_row        (fc_ibuf_load_row),
    .mlp_load_k_word     (fc_ibuf_load_k_word),
    .mlp_load_data       (fc_ibuf_load_data),

    // MLP L1 Y capture: direct from MMU output (no DSU, purely internal)
    .mlp_capture_en      (fc_ibuf_l1_capture_en),
    .mlp_col_wr          (fc_ibuf_l1_col_wr),
    .mlp_l1_out          (mmu_out),

    // Sub-cycle for MLP read
    .sub_cycle           (fc_mmu_sub_cycle),

    // Read → MMU
    .data_out            (ubuf_in_out)
);

// =============================================================================
// Instance: mmu_top
// =============================================================================
mmu_top u_mmu (
    .clk       (clk),
    .rst_n     (rst_n),
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

endmodule
