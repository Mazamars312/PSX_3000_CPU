/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module block_muldiv(
    input               clk,
    input               rst_n,

    input       [6:0]   exe_cmd_for_muldiv,
    input       [31:0]  exe_a,
    input       [31:0]  exe_b,
    input       [4:0]   exe_instr_rd,
    
    output              muldiv_busy,

    
    input               rf_mthi,
    input               rf_mtlo,
    input               rf_mult,
    input               rf_multu,
    input               rf_div,
    input               rf_divu,

    output reg  [31:0]  multi_hi,
    output reg  [31:0]  multi_lo
); /* verilator public_module */

//------------------------------------------------------------------------------ lo, hi, busy


always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                               multi_hi <= 32'd0;
    else if(rf_mthi)                                multi_hi <= exe_a;
    else if(mult_ready)                             multi_hi <= mult_result[63:32];
    else if(div_ready && div_busy)                  multi_hi <= div_remainder;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                               multi_lo <= 32'd0;
    else if(rf_mtlo)                                multi_lo <= exe_a;
    else if(mult_ready)                             multi_lo <= mult_result[31:0];
    else if(div_ready && div_busy)                  multi_lo <= div_quotient;
end

reg busy;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)   busy <= `FALSE;
    else                busy <= mult_busy || div_busy;                                       
end

wire muldiv_busy_start = |{rf_multu, rf_mult, rf_divu, rf_div};

assign muldiv_busy = muldiv_busy_start || busy;

//------------------------------------------------------------------------------ multiply

wire mult_busy = mult_counter > 2'd0;
wire mult_ready= mult_counter == 2'd1;

reg [1:0] mult_counter;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)               mult_counter <= 2'd0;
    else if(rf_mult || rf_multu)    mult_counter <= 2'd3;
    else if(mult_counter != 2'd0)   mult_counter <= mult_counter - 2'd1;
end

wire [65:0] mult_result;

model_mult
#(
    .widtha     (33),
    .widthb     (33),
    .widthp     (66)
)
model_mult_inst(
    .reset_l    (rst_n),
    .clk        (clk),
    .a          ((rf_mult)? { exe_a[31], exe_a[31:0] } : { 1'b0, exe_a[31:0] }),
    .b          ((rf_mult)? { exe_b[31], exe_b[31:0] } : { 1'b0, exe_b[31:0] }),
    .out        (mult_result)
);

//------------------------------------------------------------------------------ divide

reg div_busy;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)           div_busy <= `FALSE;
    else if(rf_div || rf_divu)  div_busy <= `TRUE;
    else if(div_ready)          div_busy <= `FALSE;
end

wire        div_ready;
wire [31:0] div_quotient;
wire [31:0] div_remainder;
        
block_long_div block_long_div_inst(
    .clk        (clk),
    .rst_n      (rst_n),
    
    .start      (rf_div || rf_divu),  //input
    .dividend   ({ rf_div & exe_a[31], exe_a[31:0] }),               //input [32:0]
    .divisor    ({ rf_div & exe_b[31], exe_b[31:0] }),               //input [32:0]
    
    .ready      (div_ready),                                                    //output
    .quotient   (div_quotient),                                                 //output [31:0]
    .remainder  (div_remainder)                                                 //output [31:0]
);

endmodule
