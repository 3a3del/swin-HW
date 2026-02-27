// =============================================================================
// output_memory.sv
// Output Memory — on-chip SRAM holding the final computation results.
//
// ── Address Map (time-shared) ─────────────────────────────────────────────
//   Conv mode : 56×56×96 = 301,056 output words   [0 .. 301,055]
//               Layout: kernel*3136 + row_group*56 + chunk*7 + col
//
//   MLP  mode : 3136×96  =  301,056 output words  [0 .. 301,055]
//               Layout: l2_col*3136 + row_grp*7 + wb_cnt
//
//   Maximum depth = 301,056 words → AW = 19  (ceil log2 = 18.2, use 19)
//
// ── Interface ────────────────────────────────────────────────────────────
//   Write port  : driven by the DSU on behalf of the active controller.
//   Read port   : CPU / DMA reads results after engine finishes.
//   Write latency : single-cycle (combinatorial write enable).
// =============================================================================

module output_memory #(
    parameter DEPTH = 301056,   // words
    parameter AW    = 19        // ceil(log2(301056)) = 19
)(
    input  logic          clk,
    input  logic          rst_n,

    // ── Write port (from DSU / output buffer) ─────────────────────────────
    input  logic [AW-1:0] wr_addr,
    input  logic [31:0]   wr_data,
    input  logic          wr_en,

    // ── Read port (CPU / DMA result readback) ─────────────────────────────
    input  logic [AW-1:0] rd_addr,
    input  logic          rd_en,
    output logic [31:0]   rd_data
);

    logic [31:0] mem [0:DEPTH-1];

    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    end

    // ── Write (synchronous) ───────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // ── Read (registered, 1-cycle latency) ────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data <= '0;
        else if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
