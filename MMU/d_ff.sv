// d_ff.sv
module ff (clk, rst_n, d, flush, q);
    input clk;
    input rst_n;
    input flush;
    input [31:0] d;
    output reg [31:0] q;

   always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            q <= 0; 
        else
            q <= d;  
      end
    
   always @(negedge clk) begin
        if (flush) 
            q <= 0;
      end    
endmodule
