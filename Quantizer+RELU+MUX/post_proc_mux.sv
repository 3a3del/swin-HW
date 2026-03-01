// =============================================================================
// post_proc_mux.sv  —  Post-Processing Multiplexer
//
// 2-to-1 multiplexer sitting at the end of the activation pipeline.
// Allows the system to choose whether the final value written to
// output_memory has been passed through ReLU or is taken directly
// from the rounding_shifter (quantizer) without activation.
//
// ── Port descriptions ─────────────────────────────────────────────────────
//   relu_in   : output of the ReLU module  (activated path)
//   quant_in  : output of rounding_shifter (bypass / linear path)
//   relu_en   : control signal from the top level
//                 1 → select relu_in   (ReLU activated output)
//                 0 → select quant_in  (raw quantizer output, no activation)
//   data_out  : selected value forwarded to output_memory write data bus
//
// ── Timing ───────────────────────────────────────────────────────────────
//   Purely combinatorial — zero added latency.
// =============================================================================

module post_proc_mux #(
    parameter int W = 32
)(
    input  logic signed [W-1:0] relu_in,    // from relu module
    input  logic signed [W-1:0] quant_in,  // from rounding_shifter (quantizer)
    input  logic                relu_en,    // 1 = use ReLU output, 0 = bypass
    output logic signed [W-1:0] data_out
);

    assign data_out = relu_en ? relu_in : quant_in;

endmodule
