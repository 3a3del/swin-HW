// =============================================================================
// output_buffer.sv
// Captures the 7 × 32-bit outputs from one MMU computation and exposes
// them word-by-word for sequential write-back to the output memory.
// =============================================================================

module output_buffer #(
    parameter N_OUT = 7
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        capture_en,
    input  logic [31:0] mmu_out [0:N_OUT-1],
    input  logic [2:0]  rd_idx,
    output logic [31:0] rd_data
);

    logic [31:0] buf [0:N_OUT-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N_OUT; i++) buf[i] <= '0;
        end else if (capture_en) begin
            for (int i = 0; i < N_OUT; i++) buf[i] <= mmu_out[i];
        end
    end

    assign rd_data = buf[rd_idx];

endmodule
