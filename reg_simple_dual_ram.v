/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2019 Murray Aickin
 */

module reg_simple_dual_ram(
    input                       clk,
    
    input       [widthad-1:0]   address_a,
    output reg     [width-1:0]  q_a,
    
    input       [widthad-1:0]   address_b,
    input                       wren_b,
    input       [width-1:0]     data_b
); /* verilator public_module */

parameter width     = 1;
parameter widthad   = 1;

integer k;
initial
begin
for (k = 0; k < (2**widthad); k = k + 1)
begin
    mem[k] = 0;
end
end

reg [width-1:0] mem [(2**widthad)-1:0];

always @(posedge clk) begin
    if(wren_b) mem[address_b] <= data_b; 
    q_a <= (wren_b && address_b == address_a) ? data_b : mem[address_a];
//    q_a <= mem[address_a];
end


endmodule
