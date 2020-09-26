/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2020 Murray Aickin
 */

module model_mult(
    input                           clk,
    input                           reset_l,
    input signed    [widtha-1:0]    a,
    input signed    [widthb-1:0]    b,
    output reg      [widthp-1:0]    out
);

//------------------------------------------------------------------------------

parameter widtha = 1;
parameter widthb = 1;
parameter widthp = 2;

//------------------------------------------------------------------------------

reg signed [widtha-1:0] a_reg;
reg signed [widthb-1:0] b_reg;
reg signed [widthp-1:0] out_1;

//assign out = out_1;

wire signed [widthp-1:0] mult_out;

reg clk_half;

always @ (posedge clk or negedge reset_l) begin // we half the clock here to increase the Mutli core, We then store on the 2nd clock cycle to the High low regs
    if (reset_l) clk_half <= 'b0;
    else clk_half <= ~clk_half;
end


always @ (posedge clk_half or negedge reset_l)
begin
    if (~reset_l) begin
        out     <= 'b0;
    end
    else begin
        out     <= a * b;
    end
end

//------------------------------------------------------------------------------

endmodule
