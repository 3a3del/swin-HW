module mmu_valid_ctrl (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [2:0]  op_code,
    input  logic [1:0]  stage,
    output logic        flush,
    output logic        valid_out
);

logic [6:0] count_mod;
logic [6:0] counter;
logic       busy;
logic       valid_d;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) valid_d <= 0;
    else        valid_d <= valid_in;
end

always_comb begin
    count_mod = 1;
    case(op_code)
        3'd1, 3'd5: begin
            count_mod = 2 << stage;
            if (op_code[2])
                count_mod = count_mod * 4;
        end
        3'd3: count_mod = 2;
    endcase
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter   <= 0;
        busy      <= 0;
        valid_out <= 0;
    end else begin
        counter   <= 0;
        valid_out <= 0;
        if (valid_in) begin
            if (op_code == 3'd0 || op_code == 3'd2) begin
                valid_out <= valid_d;
                busy      <= 0;
            end else begin
                if (busy && counter == count_mod-1) begin
                    valid_out <= 1;
                    counter   <= 0;
                    busy      <= 0;
                    if (valid_in) begin
                        counter <= 0;
                        busy    <= 1;
                    end
                end else if (busy) begin
                    counter <= counter + 1;
                end else if (valid_in) begin
                    busy    <= 1;
                    counter <= 0;
                end
            end
        end else
            busy <= 0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) flush <= 1'b0;
    else        flush <= 1'b0;
end

always @(negedge clk or negedge rst_n) begin
    if (!rst_n)      flush <= 1'b0;
    else if (valid_out) flush <= 1'b1;
end
endmodule
