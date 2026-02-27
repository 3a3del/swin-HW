// PE_block.sv
module PE_block (in, w, bias, out);
    input   [7:0] in [0:6][0:3];
    input   [7:0] w [0:3];
    input   [31:0] bias;
    output  [31:0] out [0:6];
    
    wire [31:0] mac_out [0:6][0:3];
    genvar i,j;
    generate
        for (i=0; i<7 ; i++) begin
            for (j=0; j<4 ; j++) begin
                if (j==0) begin
                    mac m (.in_a(in[i][j]), .in_b(w[j]), .in_c(bias), .out(mac_out[i][j]));
                end else begin
                    mac m (.in_a(in[i][j]), .in_b(w[j]), .in_c(mac_out[i][j-1]), .out(mac_out[i][j]));
                end
            end
            assign out[i]=mac_out[i][3];
        end
    endgenerate
endmodule
