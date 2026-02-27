// =============================================================================
// unified_weight_buf.sv  (original, unchanged)
// =============================================================================
module unified_weight_buf #(
    parameter MAX_BYTES = 384,
    parameter N_PE      = 12,
    parameter N_TAP     = 4
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        mode,
    input  logic        swap,

    input  logic        conv_load_en,
    input  logic [3:0]  conv_load_pe_idx,
    input  logic [31:0] conv_load_data,

    input  logic        conv_bias_load_en,
    input  logic [31:0] conv_bias_load_data,

    input  logic        mlp_load_en,
    input  logic [6:0]  mlp_load_k_word,
    input  logic [31:0] mlp_load_data,

    input  logic [2:0]  sub_cycle,

    output logic [7:0]  w_out   [0:N_PE-1][0:N_TAP-1],
    output logic [31:0] bias_out
);

    logic [7:0]  bank  [0:1][0:MAX_BYTES-1];
    logic [31:0] bias  [0:1];
    logic        active;
    logic        shadow;
    assign shadow = ~active;

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) active <= 1'b0;
        else if (swap) active <= shadow;

    always_ff @(posedge clk) begin
        if (mode == 1'b0) begin
            if (conv_load_en) begin
                bank[shadow][conv_load_pe_idx * N_TAP    ] <= conv_load_data[ 7: 0];
                bank[shadow][conv_load_pe_idx * N_TAP + 1] <= conv_load_data[15: 8];
                bank[shadow][conv_load_pe_idx * N_TAP + 2] <= conv_load_data[23:16];
                bank[shadow][conv_load_pe_idx * N_TAP + 3] <= conv_load_data[31:24];
            end
            if (conv_bias_load_en)
                bias[shadow] <= conv_bias_load_data;
        end else begin
            if (mlp_load_en) begin
                bank[shadow][mlp_load_k_word * N_TAP    ] <= mlp_load_data[ 7: 0];
                bank[shadow][mlp_load_k_word * N_TAP + 1] <= mlp_load_data[15: 8];
                bank[shadow][mlp_load_k_word * N_TAP + 2] <= mlp_load_data[23:16];
                bank[shadow][mlp_load_k_word * N_TAP + 3] <= mlp_load_data[31:24];
            end
        end
    end

    always_comb begin
        if (mode == 1'b0) begin
            for (int pe = 0; pe < N_PE; pe++)
                for (int tap = 0; tap < N_TAP; tap++)
                    w_out[pe][tap] = bank[active][pe * N_TAP + tap];
            bias_out = bias[active];
        end else begin
            for (int pe = 0; pe < N_PE; pe++)
                for (int tap = 0; tap < N_TAP; tap++) begin
                    automatic int k = int'(sub_cycle) * (N_PE * N_TAP) + pe * N_TAP + tap;
                    w_out[pe][tap] = (k < MAX_BYTES) ? bank[active][k] : 8'b0;
                end
            bias_out = 32'd0;
        end
    end

endmodule
