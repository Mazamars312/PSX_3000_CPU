/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */



module pipeline_rf(
    input                   clk,
    input                   rst_n,
    
    input                   rf_stall,
    
    input       [15:0]      exe_instant_value,
    
    input       [1:0]       exe_a_bypass,
    input       [2:0]       exe_b_bypass,
    
    // EXE Bypass core
    input       [31:0]      mem_result,
    // MEM Write back
    input       [4:0]       wb_result_index,
    input       [31:0]      wb_result,
    // RF Output
    input       [4:0]       rf_address_a,
    input       [4:0]       rf_address_b,
    output reg  [31:0]      exe_a,
    output reg  [31:0]      exe_b
    
    );
    `include "defines.v"
    reg [31:0]  mtp_result;
    reg [4:0]   address_a_reg;
    reg [4:0]   address_b_reg;
    
    wire    [31:0]  q_a;
    wire    [31:0]  q_b;

// The mtp_result is for the bypass on the write back as bram takes one clock to write data

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) mtp_result <= 'b0;
        else mtp_result <= wb_result;
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            address_a_reg <= 'b0;
            address_b_reg <= 'b0;
            end
        else if (~rf_stall) begin
            address_a_reg <= rf_address_a;
            address_b_reg <= rf_address_b;
            end
    end

// rf_a bypass

    always @* begin
        case(exe_a_bypass)
            `rf_exe_bypass      : exe_a <= mem_result;
            `rf_mem_bypass      : exe_a <= wb_result;
//            `rf_rb_bypass       : exe_a <= mtp_result;
            default             : exe_a <= q_a;
        endcase
    end

// rf_b bypass

    always @* begin
        case(exe_b_bypass)
            `rf_inst_bypass     : exe_b <= {{16{exe_instant_value[15]}}, exe_instant_value};
            `rf_exe_bypass      : exe_b <= mem_result;
            `rf_mem_bypass      : exe_b <= wb_result;
//            `rf_rb_bypass       : exe_b <= mtp_result;
            default             : exe_b <= q_b;
        endcase
    end

/*
    The REG Core
*/

    reg_simple_dual_ram #(
        .width          (32),
        .widthad        (5)
    )
    regs_a_inst(
        .clk            (clk),
        
        .address_a      ((rf_stall)? address_a_reg : rf_address_a),
        .q_a            (q_a),
        
        .address_b      (wb_result_index),
        .wren_b         (wb_result_index != 5'd0),
        .data_b         (wb_result)
    );
    
    reg_simple_dual_ram #(
        .width          (32),
        .widthad        (5)
    )
    regs_b_inst(
        .clk            (clk),
        
        .address_a      ((rf_stall)? address_b_reg : rf_address_b),
        .q_a            (q_b),
        
        .address_b      (wb_result_index),
        .wren_b         (wb_result_index != 5'd0),
        .data_b         (wb_result)
    );


//------------------------------------------------------------------------------

endmodule
