// =============================================================================
// relu.sv  —  Rectified Linear Unit
//
// Combinatorial module that clips negative values to zero.
// Data is treated as a signed 32-bit fixed-point value matching the
// rounding_shifter output format.
//
// ── Truth table ──────────────────────────────────────────────────────────
//   in_value >= 0  →  out_value = in_value
//   in_value <  0  →  out_value = 0
//
// ── Timing ───────────────────────────────────────────────────────────────
//   Purely combinatorial — zero added latency.
// =============================================================================

module relu #(
    parameter int W = 32        // data width (must match rounding_shifter W_INPUT)
)(
    input  logic signed [W-1:0] in_value,
    output logic signed [W-1:0] out_value
);

    // Clip negative values to zero.
    // The MSB of a signed value is '1' when the number is negative.
    assign out_value = in_value[W-1] ? {W{1'b0}} : in_value;

endmodule
