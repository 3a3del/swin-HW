// =============================================================================
// unified_input_buf.sv  (original, unchanged)
// =============================================================================
module unified_input_buf #(
    parameter N_ROWS   = 7,
    parameter K_MAX    = 384,
    parameter N_PE     = 12,
    parameter N_WIN    = 7,
    parameter N_TAP    = 4
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        mode,
    input  logic        swap,

    input  logic        conv_load_en,
    input  logic [3:0]  conv_load_pe_idx,
    input  logic [2:0]  conv_load_win_idx,
    input  logic [31:0] conv_load_data,

    input  logic        mlp_load_en,
    input  logic [2:0]  mlp_load_row,
    input  logic [6:0]  mlp_load_k_word,
    input  logic [31:0] mlp_load_data,

    input  logic        mlp_capture_en,
    input  logic [8:0]  mlp_col_wr,
    input  logic [31:0] mlp_l1_out [0:N_ROWS-1],

    input  logic [2:0]  sub_cycle,

    output logic [7:0]  data_out [0:N_PE-1][0:N_WIN-1][0:N_TAP-1]
);

    localparam BANK_BYTES = N_ROWS * K_MAX;  // 2688

    logic [7:0] bank [0:1][0:BANK_BYTES-1];
    logic       active;
    logic       shadow;
    assign shadow = ~active;

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) active <= 1'b0;
        else if (swap) active <= shadow;

    always_ff @(posedge clk) begin
        if (mode == 1'b0) begin
            if (conv_load_en) begin
                automatic int base = int'(conv_load_pe_idx) * (N_WIN * N_TAP)
                                   + int'(conv_load_win_idx) * N_TAP;
                bank[shadow][base    ] <= conv_load_data[ 7: 0];
                bank[shadow][base + 1] <= conv_load_data[15: 8];
                bank[shadow][base + 2] <= conv_load_data[23:16];
                bank[shadow][base + 3] <= conv_load_data[31:24];
            end
        end else begin
            if (mlp_load_en) begin
                automatic int base = int'(mlp_load_row) * K_MAX
                                   + int'(mlp_load_k_word) * N_TAP;
                bank[shadow][base    ] <= mlp_load_data[ 7: 0];
                bank[shadow][base + 1] <= mlp_load_data[15: 8];
                bank[shadow][base + 2] <= mlp_load_data[23:16];
                bank[shadow][base + 3] <= mlp_load_data[31:24];
            end
            if (mlp_capture_en) begin
                for (int r = 0; r < N_ROWS; r++) begin
                    automatic int addr = r * K_MAX + int'(mlp_col_wr);
                    bank[shadow][addr] <= mlp_l1_out[r][7:0];
                end
            end
        end
    end

    always_comb begin
        if (mode == 1'b0) begin
            for (int pe = 0; pe < N_PE; pe++)
                for (int win = 0; win < N_WIN; win++)
                    for (int tap = 0; tap < N_TAP; tap++)
                        data_out[pe][win][tap] =
                            bank[active][pe * (N_WIN * N_TAP) + win * N_TAP + tap];
        end else begin
            for (int pe = 0; pe < N_PE; pe++)
                for (int win = 0; win < N_WIN; win++)
                    for (int tap = 0; tap < N_TAP; tap++) begin
                        automatic int k = int'(sub_cycle) * (N_PE * N_TAP)
                                        + pe * N_TAP + tap;
                        automatic int addr = win * K_MAX + k;
                        data_out[pe][win][tap] =
                            (k < K_MAX) ? bank[active][addr] : 8'b0;
                    end
        end
    end

endmodule
