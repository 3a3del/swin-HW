module adder_tree #(parameter N=12, parameter K=15)(in,sum);
input  [K-1:0] in [0:N-1];
output [K-1:0] sum;

localparam stages = $clog2(N);
wire [K-1:0] stage_in [0:stages][0:N-1];

genvar i,level;
generate
    for (i=0; i<N ; i=i+1) begin
        assign stage_in[0][i] = in[i];
    end    
endgenerate 

generate 
    for (level = 0; level < stages; level = level + 1) begin : STAGE
        for (i = 0; i < N; i = i + (2<<level)) begin : ADDER
            if (i + (1 << level) < N)
                assign stage_in[level+1][i] = stage_in[level][i] + stage_in[level][i + (1 << level)];
            else
                assign stage_in[level+1][i] = stage_in[level][i];
        end
    end
endgenerate

assign sum = stage_in[stages][0];
endmodule
