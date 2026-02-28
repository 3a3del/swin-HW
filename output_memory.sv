// =============================================================================
// output_memory.sv  (rev 3)
//
// ── What changed from rev 2 ───────────────────────────────────────────────
// wr_data is now driven DIRECTLY by the post-processing mux output
// (post_proc_data) in full_system_top — it no longer travels through the DSU.
// The DSU still provides wr_addr and wr_en, but the data bus is decoupled.
//
// ── Three-port interface ──────────────────────────────────────────────────
//
//   Port 1 — ENGINE WRITE
//     wr_addr  : from DSU  (address computed by active controller FSM)
//     wr_en    : from DSU  (asserted during WRITEBACK FSM state)
//     wr_data  : from post_proc_mux  (post_proc_data — DIRECT, not via DSU)
//
//   Port 2 — CPU / DMA READ  (result readback after done)
//     cpu_rd_addr / cpu_rd_en / cpu_rd_data  — unchanged
//
//   Port 3 — ENGINE FEEDBACK READ  (layer chaining)
//     fb_rd_addr / fb_rd_en : from DSU  (redirected controller input address)
//     fb_rd_data            : to DSU    (returned to controller as input data)
//     1-cycle registered latency, identical to fib_memory.
//
// ── Address map (unchanged) ───────────────────────────────────────────────
//   Conv mode : 56×56×96  = 301,056 words  [0 .. 301,055]
//   MLP  mode : 3136×96   = 301,056 words  [0 .. 301,055]
//   AW = 19
// =============================================================================

module output_memory #(
    parameter DEPTH = 301056,
    parameter AW    = 19
)(
    input  logic          clk,
    input  logic          rst_n,

    // ── Port 1 : Engine write ──────────────────────────────────────────────
    input  logic [AW-1:0]  wr_addr,
    input  logic [31:0]    wr_data,   // connected directly to post_proc_data
    input  logic           wr_en,     // from DSU

    // ── Port 2 : CPU / DMA read ───────────────────────────────────────────
    input  logic [AW-1:0]  cpu_rd_addr,
    input  logic           cpu_rd_en,
    output logic [31:0]    cpu_rd_data,

    // ── Port 3 : Engine feedback read ─────────────────────────────────────
    input  logic [AW-1:0]  fb_rd_addr,
    input  logic           fb_rd_en,
    output logic [31:0]    fb_rd_data
);

    logic [31:0] mem [0:DEPTH-1];

    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    end

    // ── Write ─────────────────────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // ── CPU read (1-cycle latency) ─────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)           cpu_rd_data <= '0;
        else if (cpu_rd_en)   cpu_rd_data <= mem[cpu_rd_addr];
    end

    // ── Feedback read (1-cycle latency, same as fib_memory) ───────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)           fb_rd_data <= '0;
        else if (fb_rd_en)    fb_rd_data <= mem[fb_rd_addr];
    end

endmodule