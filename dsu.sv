// =============================================================================
// dsu.sv  —  Data Selection Unit
//
// The DSU sits between the two large memories (weight_memory, fib_memory,
// output_memory) and the two controllers + buffers.  It is the single point
// of arbitration / routing for all memory traffic.
//
// ── Responsibilities ──────────────────────────────────────────────────────
//  1. ADDRESS MUX
//       mode=0 (Conv) → present conv_controller addresses to both memories.
//       mode=1 (MLP)  → present fc_controller addresses to both memories.
//       Since the two modes never overlap, a simple combinatorial mux suffices.
//
//  2. WEIGHT MEMORY UNIFICATION (MLP mode)
//       fc_controller exposes separate w1mem / w2mem ports.
//       The DSU merges them into a single weight_memory access:
//         W1 region : [    0 .. W1_WORDS-1 ]   (9,216 words)
//         W2 region : [W1_WORDS .. W1+W2-1 ]   (9,216 words, base = 9216)
//       Address offset is added here, transparent to fc_controller.
//
//  3. DATA ROUTING (rd_data)
//       weight_memory.rd_data → back to the requesting controller port.
//       fib_memory.rd_data    → back to the requesting controller port.
//       Both memories share a single rd_data bus per memory; the controller
//       that issued the request knows to capture the result one cycle later.
//
//  4. OUTPUT MEMORY WRITE
//       mode=0 → conv_omem_addr / conv_omem_wr_en / obuf_rd_data
//       mode=1 → fc_omem_addr  / fc_omem_wr_en  / obuf_rd_data
//       omem_wr_data is always the output-buffer read-data (obuf_rd_data).
//
// ── Memory address map (time-shared, CPU loads correct data before start) ──
//   weight_memory:
//     Conv   :   0 .. 1247    (96k × 13w)
//     MLP W1 :   0 .. 9215    (384c × 24w)  ← same base as Conv (time-shared)
//     MLP W2 : 9216 .. 18431  (96c  × 96w)  ← base offset added by DSU
//
//   fib_memory:
//     Conv   :   0 .. 37631   (3ch × 224×56w)
//     MLP X  :   0 .. 75263   (3136r × 24w)  ← same base (time-shared)
// =============================================================================

module dsu #(
    parameter WAW = 15,     // weight_memory address width
    parameter FAW = 17,     // fib_memory address width
    parameter OAW = 19,     // output_memory address width

    // MLP weight base offsets inside weight_memory
    parameter MLP_W2_BASE = 32'd9216   // W2 starts after W1 (384cols × 24w)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        mode,       // 0 = Conv, 1 = MLP

    // =========================================================================
    // CONV CONTROLLER PORTS
    // =========================================================================

    // — Conv weight memory interface ──────────────────────────────────────────
    input  logic [31:0] conv_wmem_addr,
    input  logic        conv_wmem_rd_en,
    output logic [31:0] conv_wmem_rd_data,   // returned to conv_controller

    // — Conv image (FIB) memory interface ─────────────────────────────────────
    input  logic [31:0] conv_imem_addr,
    input  logic        conv_imem_rd_en,
    output logic [31:0] conv_imem_rd_data,   // returned to conv_controller

    // — Conv output memory write ───────────────────────────────────────────────
    input  logic [31:0] conv_omem_addr,
    input  logic        conv_omem_wr_en,

    // =========================================================================
    // FC CONTROLLER PORTS
    // =========================================================================

    // — MLP W1 weight memory interface ────────────────────────────────────────
    input  logic [31:0] fc_w1mem_addr,
    input  logic        fc_w1mem_rd_en,
    output logic [31:0] fc_w1mem_rd_data,    // returned to fc_controller

    // — MLP W2 weight memory interface ────────────────────────────────────────
    input  logic [31:0] fc_w2mem_addr,
    input  logic        fc_w2mem_rd_en,
    output logic [31:0] fc_w2mem_rd_data,    // returned to fc_controller

    // — MLP X (FIB) memory interface ──────────────────────────────────────────
    input  logic [31:0] fc_xmem_addr,
    input  logic        fc_xmem_rd_en,
    output logic [31:0] fc_xmem_rd_data,     // returned to fc_controller

    // — MLP output memory write ────────────────────────────────────────────────
    input  logic [31:0] fc_omem_addr,
    input  logic        fc_omem_wr_en,

    // =========================================================================
    // SHARED OUTPUT BUFFER DATA (write-back bus, same for both modes)
    // =========================================================================
    input  logic [31:0] obuf_rd_data,        // from output_buffer.rd_data

    // =========================================================================
    // PHYSICAL MEMORY PORTS  (connect to weight_memory / fib_memory / output_memory)
    // =========================================================================

    // — weight_memory ──────────────────────────────────────────────────────────
    output logic [WAW-1:0] wmem_rd_addr,
    output logic           wmem_rd_en,
    input  logic [31:0]    wmem_rd_data,

    // — fib_memory ─────────────────────────────────────────────────────────────
    output logic [FAW-1:0] fib_rd_addr,
    output logic           fib_rd_en,
    input  logic [31:0]    fib_rd_data,

    // — output_memory ──────────────────────────────────────────────────────────
    output logic [OAW-1:0] omem_wr_addr,
    output logic [31:0]    omem_wr_data,
    output logic           omem_wr_en
);

    // =========================================================================
    // 1.  WEIGHT MEMORY ADDRESS MUX
    // =========================================================================
    // Track which MLP weight request is in flight so we can steer rd_data back.
    // We need a 1-cycle delayed version of the w1/w2 select to align with the
    // registered memory output.
    logic        mlp_w2_sel;       // combinatorial: W2 request active this cycle
    logic        mlp_w2_sel_d;     // registered:    W2 request was active last cycle

    assign mlp_w2_sel = fc_w2mem_rd_en && !fc_w1mem_rd_en;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) mlp_w2_sel_d <= 1'b0;
        else        mlp_w2_sel_d <= mlp_w2_sel;
    end

    always_comb begin
        if (mode == 1'b0) begin
            // ── Conv: direct address, no offset ──────────────────────────
            wmem_rd_addr = conv_wmem_addr[WAW-1:0];
            wmem_rd_en   = conv_wmem_rd_en;
        end else begin
            // ── MLP: W1 at base 0, W2 at base MLP_W2_BASE ───────────────
            if (fc_w2mem_rd_en) begin
                wmem_rd_addr = (fc_w2mem_addr + MLP_W2_BASE)[WAW-1:0];
                wmem_rd_en   = 1'b1;
            end else begin
                wmem_rd_addr = fc_w1mem_addr[WAW-1:0];
                wmem_rd_en   = fc_w1mem_rd_en;
            end
        end
    end

    // ── Weight memory rd_data routing ────────────────────────────────────────
    // Delayed select decides which controller port gets the result.
    always_comb begin
        conv_wmem_rd_data = '0;
        fc_w1mem_rd_data  = '0;
        fc_w2mem_rd_data  = '0;

        if (mode == 1'b0) begin
            conv_wmem_rd_data = wmem_rd_data;
        end else begin
            if (mlp_w2_sel_d)
                fc_w2mem_rd_data = wmem_rd_data;
            else
                fc_w1mem_rd_data = wmem_rd_data;
        end
    end

    // =========================================================================
    // 2.  FIB MEMORY ADDRESS MUX
    // =========================================================================
    // Both Conv (image) and MLP (X) use base address 0 inside fib_memory
    // (time-shared: CPU loads the right data before each mode starts).

    always_comb begin
        if (mode == 1'b0) begin
            fib_rd_addr = conv_imem_addr[FAW-1:0];
            fib_rd_en   = conv_imem_rd_en;
        end else begin
            fib_rd_addr = fc_xmem_addr[FAW-1:0];
            fib_rd_en   = fc_xmem_rd_en;
        end
    end

    // ── FIB memory rd_data routing ───────────────────────────────────────────
    always_comb begin
        conv_imem_rd_data = '0;
        fc_xmem_rd_data   = '0;

        if (mode == 1'b0)
            conv_imem_rd_data = fib_rd_data;
        else
            fc_xmem_rd_data   = fib_rd_data;
    end

    // =========================================================================
    // 3.  OUTPUT MEMORY WRITE MUX
    // =========================================================================
    always_comb begin
        if (mode == 1'b0) begin
            omem_wr_addr = conv_omem_addr[OAW-1:0];
            omem_wr_en   = conv_omem_wr_en;
        end else begin
            omem_wr_addr = fc_omem_addr[OAW-1:0];
            omem_wr_en   = fc_omem_wr_en;
        end
        omem_wr_data = obuf_rd_data;   // always from output buffer
    end

endmodule
