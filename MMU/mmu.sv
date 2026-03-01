module mmu (clk, rst_n, flush, mmu_in, mmu_w, mmu_bias, mmu_out);

    input clk, rst_n;
    input  [7:0]  mmu_in     [0:11][0:6][0:3];
    input  [7:0]  mmu_w      [0:11][0:3];
    input  [31:0] mmu_bias   [0:11];
    input         flush;
    output [31:0] mmu_out    [0:6];

    wire [31:0] pe_out       [0:11][0:6];
    wire [31:0] buff_out     [0:11][0:6];
    wire [31:0] acc_res      [0:11][0:6];
    wire [31:0] tree_in      [0:6][0:11];

    genvar i,j;
    generate
        for (i=0; i<12; i++) begin: PE
            if (i==0) 
                PE_block PE (.in(mmu_in[i]), .w(mmu_w[i]), .bias(mmu_bias[i]), .out(pe_out[i]));
            else 
                PE_block PE (.in(mmu_in[i]), .w(mmu_w[i]), .bias(0), .out(pe_out[i]));
        end
        for (i=0; i<12; i++) begin: Buff
            for (j=0; j<7; j++) begin
                ff buff (.clk(clk), .rst_n(rst_n), .flush(flush), .d(acc_res[i][j]), .q(buff_out[i][j]));
                assign acc_res[i][j] = pe_out[i][j] + buff_out[i][j];
            end
        end
        for (i=0; i<7; i++) begin
            for (j=0; j<12; j++) begin
                assign tree_in[i][j] = buff_out[j][i];
            end
            adder_tree #(.N(12), .K(32)) adder_tree (.in(tree_in[i]), .sum(mmu_out[i]));
        end
    endgenerate
    
endmodule
