// =============================================================================
// dsu.sv  (rev 3)
//
// ── What changed from rev 2 ───────────────────────────────────────────────
// The DSU no longer carries or forwards the output write DATA.
// Previously obuf_rd_data was an input that the DSU forwarded to omem_wr_data.
// That path is now a direct wire in full_system_top:
//
//     post_proc_data ──────────────────────► output_memory.wr_data
//
// The DSU is still responsible for the write ADDRESS and write ENABLE
// (omem_wr_addr, omem_wr_en), and for the feedback read path.
// Removed ports:  obuf_rd_data (input),  omem_wr_data (output).
//
// ── Responsibilities (updated) ────────────────────────────────────────────
//  1. Weight memory address mux          (Conv vs MLP, W1 vs W2 offset)
//  2. Input data SOURCE mux              (fib_memory  vs  output_memory fb)
//       omem_fb_en=0 → normal:    controller addr → fib_memory
//       omem_fb_en=1 → feedback:  controller addr → output_memory fb port
//  3. Output memory WRITE ADDRESS+ENABLE (data comes directly from mux)
//  4. Output memory FEEDBACK READ        (addr+en out, data in → controller)
// =============================================================================

module dsu #(
    parameter WAW         = 15,
    parameter FAW         = 17,
    parameter OAW         = 19,
    parameter MLP_W2_BASE = 32'd9216
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        mode,           // 0 = Conv, 1 = MLP
    input  logic        omem_fb_en,     // 1 = feedback path active

    // =========================================================================
    // CONV CONTROLLER PORTS
    // =========================================================================
    input  logic [31:0] conv_wmem_addr,
    input  logic        conv_wmem_rd_en,
    output logic [31:0] conv_wmem_rd_data,

    input  logic [31:0] conv_imem_addr,
    input  logic        conv_imem_rd_en,
    output logic [31:0] conv_imem_rd_data,

    input  logic [31:0] conv_omem_addr,
    input  logic        conv_omem_wr_en,

    // =========================================================================
    // FC CONTROLLER PORTS
    // =========================================================================
    input  logic [31:0] fc_w1mem_addr,
    input  logic        fc_w1mem_rd_en,
    output logic [31:0] fc_w1mem_rd_data,

    input  logic [31:0] fc_w2mem_addr,
    input  logic        fc_w2mem_rd_en,
    output logic [31:0] fc_w2mem_rd_data,

    input  logic [31:0] fc_xmem_addr,
    input  logic        fc_xmem_rd_en,
    output logic [31:0] fc_xmem_rd_data,

    input  logic [31:0] fc_omem_addr,
    input  logic        fc_omem_wr_en,

    // =========================================================================
    // PHYSICAL MEMORY PORTS
    // =========================================================================

    // — weight_memory ──────────────────────────────────────────────────────────
    output logic [WAW-1:0] wmem_rd_addr,
    output logic           wmem_rd_en,
    input  logic [31:0]    wmem_rd_data,

    // — fib_memory ─────────────────────────────────────────────────────────────
    output logic [FAW-1:0] fib_rd_addr,
    output logic           fib_rd_en,
    input  logic [31:0]    fib_rd_data,

    // — output_memory write address + enable ONLY (data wired directly in top) ─
    output logic [OAW-1:0] omem_wr_addr,
    output logic           omem_wr_en,

    // — output_memory feedback read ────────────────────────────────────────────
    output logic [OAW-1:0] omem_fb_rd_addr,
    output logic           omem_fb_rd_en,
    input  logic [31:0]    omem_fb_rd_data
);

    // =========================================================================
    // 1.  WEIGHT MEMORY MUX  (unchanged)
    // =========================================================================
    logic mlp_w2_sel;
    logic mlp_w2_sel_d;

    assign mlp_w2_sel = fc_w2mem_rd_en && !fc_w1mem_rd_en;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) mlp_w2_sel_d <= 1'b0;
        else        mlp_w2_sel_d <= mlp_w2_sel;
    end

    always_comb begin
        if (mode == 1'b0) begin
            wmem_rd_addr = conv_wmem_addr[WAW-1:0];
            wmem_rd_en   = conv_wmem_rd_en;
        end else begin
            if (fc_w2mem_rd_en) begin
                wmem_rd_addr = (fc_w2mem_addr + MLP_W2_BASE)[WAW-1:0];
                wmem_rd_en   = 1'b1;
            end else begin
                wmem_rd_addr = fc_w1mem_addr[WAW-1:0];
                wmem_rd_en   = fc_w1mem_rd_en;
            end
        end
    end

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
    // 2.  INPUT DATA SOURCE MUX  (fib_memory  OR  output_memory feedback)
    // =========================================================================
    // 1-cycle delayed flag to align rd_data steering with registered memory output.
    logic omem_fb_en_d;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) omem_fb_en_d <= 1'b0;
        else        omem_fb_en_d <= omem_fb_en;
    end

    // Address + enable to the chosen physical memory
    always_comb begin
        fib_rd_addr     = '0;
        fib_rd_en       = 1'b0;
        omem_fb_rd_addr = '0;
        omem_fb_rd_en   = 1'b0;

        if (!omem_fb_en) begin
            // Normal path — input data from fib_memory
            if (mode == 1'b0) begin
                fib_rd_addr = conv_imem_addr[FAW-1:0];
                fib_rd_en   = conv_imem_rd_en;
            end else begin
                fib_rd_addr = fc_xmem_addr[FAW-1:0];
                fib_rd_en   = fc_xmem_rd_en;
            end
        end else begin
            // Feedback path — input data from output_memory
            if (mode == 1'b0) begin
                omem_fb_rd_addr = conv_imem_addr[OAW-1:0];
                omem_fb_rd_en   = conv_imem_rd_en;
            end else begin
                omem_fb_rd_addr = fc_xmem_addr[OAW-1:0];
                omem_fb_rd_en   = fc_xmem_rd_en;
            end
        end
    end

    // Read data back to the requesting controller (delayed flag steers correctly)
    always_comb begin
        conv_imem_rd_data = '0;
        fc_xmem_rd_data   = '0;

        if (!omem_fb_en_d) begin
            // Data came from fib_memory
            if (mode == 1'b0)
                conv_imem_rd_data = fib_rd_data;
            else
                fc_xmem_rd_data   = fib_rd_data;
        end else begin
            // Data came from output_memory feedback port
            if (mode == 1'b0)
                conv_imem_rd_data = omem_fb_rd_data;
            else
                fc_xmem_rd_data   = omem_fb_rd_data;
        end
    end

    // =========================================================================
    // 3.  OUTPUT MEMORY WRITE ADDRESS + ENABLE ONLY
    //     (wr_data is connected directly to post_proc_data in full_system_top)
    // =========================================================================
    always_comb begin
        if (mode == 1'b0) begin
            omem_wr_addr = conv_omem_addr[OAW-1:0];
            omem_wr_en   = conv_omem_wr_en;
        end else begin
            omem_wr_addr = fc_omem_addr[OAW-1:0];
            omem_wr_en   = fc_omem_wr_en;
        end
    end

endmodule