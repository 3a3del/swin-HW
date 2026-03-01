// =============================================================================
// weight_memory.sv
// Unified Weight Memory — large on-chip SRAM holding ALL weight data.
//
// ── Address Map (time-shared, only one mode active at a time) ─────────────
//   Conv mode   : 96 kernels × 13 words/kernel = 1,248 words  [0 .. 1247]
//                 Word k*13+p  → PE-p weights of kernel k  (p=0..11)
//                 Word k*13+12 → bias of kernel k
//
//   MLP  mode   : W1 (96×384, col-major)  + W2 (384×96, col-major)
//                 W1: 384 cols × 24 words/col  = 9,216 words  [0 .. 9215]
//                 W2:  96 cols × 96 words/col  = 9,216 words  [9216 .. 18431]
//                 Total MLP weight words        = 18,432
//
//   Maximum depth = 18,432 words → AW = 15  (ceil log2 = 14.17, use 15)
//
// ── Interface ────────────────────────────────────────────────────────────
//   Single write port  : loaded by external DMA / CPU before engine start.
//   Single read port   : driven by the DSU on behalf of the active controller.
//   Read latency       : 1 cycle (rd_data valid the cycle after rd_en).
// =============================================================================

module weight_memory #(
    parameter DEPTH = 18432,    // words (covers MLP case; Conv uses only 1248)
    parameter AW    = 15        // ceil(log2(18432)) = 15
)(
    input  logic          clk,
    input  logic          rst_n,

    // ── Write port (CPU / DMA) ────────────────────────────────────────────
    input  logic [AW-1:0] wr_addr,
    input  logic [31:0]   wr_data,
    input  logic          wr_en,

    // ── Read port (to DSU → controllers → weight buffers) ─────────────────
    input  logic [AW-1:0] rd_addr,
    input  logic          rd_en,
    output logic [31:0]   rd_data
);

    logic [31:0] mem [0:DEPTH-1];

    // Initialise to zero (simulation only; synthesis infers SRAM)
    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    end

    // ── Write ─────────────────────────────────────────────────────────────
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
