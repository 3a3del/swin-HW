module mac(in_a, in_b, in_c, out);
    input  [7:0] in_a;
    input  [7:0] in_b;
    input  [31:0] in_c;
    output [31:0] out;

    wire [31:0] mul_res;
    assign  mul_res = {{24{in_a[7]}},in_a} * {{24{in_b[7]}},in_b};
    assign  out = mul_res + in_c;
endmodule
