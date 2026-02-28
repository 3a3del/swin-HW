module rounding_shifter #(
    parameter int W_INPUT = 32,
    parameter int W_SHIFT = 8
)(
    input  logic signed [W_INPUT-1:0] in_value,
    input  logic signed [W_SHIFT-1:0] shift_amt,
    output logic signed [W_INPUT-1:0] out_value
);
    logic                      shift_sign;
    logic signed [W_SHIFT-1:0] abs_shift;
    logic signed [W_INPUT-1:0] temp;
    logic                      value_sign;
    logic signed [W_INPUT-1:0] abs_value;
    logic [W_INPUT-1:0]        lsb, half;
    logic                      is_greater_than_half, is_half_and_odd;

    assign shift_sign = shift_amt[W_SHIFT-1];
    assign abs_shift  = shift_sign ? -shift_amt : shift_amt;

    always_comb begin
        temp                = in_value;
        value_sign          = 1'b0;
        abs_value           = '0;
        lsb                 = '0;
        half                = '0;
        is_greater_than_half= 1'b0;
        is_half_and_odd     = 1'b0;

        if (abs_shift == 0) begin
            temp = in_value;
        end else if (!shift_sign) begin            // right shift
            value_sign = in_value[W_INPUT-1];
            abs_value  = value_sign ? -in_value : in_value;
            lsb        = abs_value & ((1 << abs_shift) - 1'b1);
            half       = {{W_INPUT-1{1'b0}}, 1'b1} << (abs_shift - 1'b1);
            is_greater_than_half = (lsb > half);
            temp = abs_value >> abs_shift;
            is_half_and_odd = (lsb == half) && temp[0];
            if (is_greater_than_half || is_half_and_odd) temp = temp + 1'b1;
            if (value_sign) temp = -temp;
        end else begin                             // left shift
            temp = in_value <<< abs_shift;
        end
    end

    assign out_value = temp;
endmodule



`timescale 1ns/1ps

module rounding_shifter_tb;

    localparam int W_INPUT = 32;
    localparam int W_SHIFT = 8;

    logic signed [W_INPUT-1:0] in_value;
    logic signed [W_SHIFT-1:0] shift_amt;
    logic signed [W_INPUT-1:0] out_value;

    rounding_shifter #(
        .W_INPUT(W_INPUT),
        .W_SHIFT(W_SHIFT)
    ) dut (
        .in_value(in_value),
        .shift_amt(shift_amt),
        .out_value(out_value)
    );

    // ==================================================
    // Reference model (OLD ModelSim compatible)
    // ==================================================
    function automatic signed [W_INPUT-1:0] ref_model;
        input signed [W_INPUT-1:0] val;
        input signed [W_SHIFT-1:0] shift;

        // ---- declarations FIRST (important) ----
        integer abs_shift;
        integer lsb;
        integer half;
        logic val_sign;
        logic signed [W_INPUT-1:0] abs_val;
        logic signed [W_INPUT-1:0] tmp;
        begin
            abs_shift = (shift < 0) ? -shift : shift;

            if (abs_shift == 0) begin
                ref_model = val;
            end
            else if (shift >= 0) begin
                // Right shift with rounding
                val_sign = val[W_INPUT-1];
                abs_val  = val_sign ? -val : val;

                lsb  = abs_val & ((1 << abs_shift) - 1);
                half = 1 << (abs_shift - 1);

                tmp = abs_val >>> abs_shift;

                if ((lsb > half) || ((lsb == half) && tmp[0]))
                    tmp = tmp + 1;

                ref_model = val_sign ? -tmp : tmp;
            end
            else begin
                // Left shift
                ref_model = val <<< abs_shift;
            end
        end
    endfunction

    // ==================================================
    // Self-checking task
    // ==================================================
    task automatic check;
        input signed [W_INPUT-1:0] val;
        input signed [W_SHIFT-1:0] shift;

        // ---- declarations FIRST ----
        logic signed [W_INPUT-1:0] expected;
        begin
            in_value  = val;
            shift_amt = shift;
            #50;

            expected = ref_model(val, shift);

            $display(
                "IN=%0d  SHIFT=%0d | OUT=%0d EXPECTED=%0d",
                val, shift, out_value, expected
            );

            if (out_value !== expected) begin
                $error(
                    "FAIL: in=%0d shift=%0d expected=%0d got=%0d",
                    val, shift, expected, out_value
                );
            end
            else begin
                $display("PASS\n");
            end
        end
    endtask

    // ==================================================
    // Test sequence
    // ==================================================
    integer i;

    initial begin
        $display("\n==== START SELF-CHECKING TB ====\n");

        check(100, 0);
        check(-100, 0);

        check(40, 2);
        check(-40, 2);

        check(22, 2);
        check(-22, 2);

        check(3, 1);
        check(5, 1);
        check(-3, 1);
        check(-5, 1);

        check(7,  -1);
        check(-7, -2);

        check(123456, 5);
        check(-123456, 5);

        $display("---- RANDOM TESTS ----\n");
        for (i = 0; i < 50; i = i + 1) begin
            check(
                ($urandom_range(0, 400000) - 200000),
                ($urandom_range(0, 40) - 20)
            );
        end

        $display("\n==== ALL TESTS FINISHED ====");
        $finish;
    end

endmodule
