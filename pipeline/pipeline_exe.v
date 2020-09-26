/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2020 Murray Aickin
 */

`include "defines.v"

module pipeline_exe(
    input               clk,
    input               rst_n,
    //
    input               exe_stall,
    input               exe_clear,
    
    input               config_kernel_mode,
    input               config_cache_enabled,
    input               config_cache_isoloation,

    input   [31:0]      exe_a,
    input   [31:0]      exe_b,
    
    input   [31:0]      exe_pc,
    
    input   [26:0]      exe_instant_value,

    input   [3:0]       exe_su_alu,
    input   [2:0]       exe_su_branch,
    input   [4:0]       exe_su_load,
    input   [4:0]       exe_su_store,
    input   [2:0]       exe_su_shift,
    input   [3:0]       exe_su_result_mux,
    input               exe_su_shift_in,
    input               exe_su_signed_alu,

    input               exe_mthi,
    input               exe_mtlo,
    input               exe_mult,
    input               exe_multu,
    input               exe_div,
    input               exe_divu,
    //
    input               exe_load,
    input               exe_store,
    //

    
    output reg  [1:0]   mem_branched,
    output reg  [31:0]  mem_branch_address,

    output reg          mem_load_address_error,
    output reg          mem_store_address_error,

    output reg          mem_int_overflow,
    output reg          mem_cache_enabled,
    //
    output reg  [31:0]  mem_result,
    output reg  [3:0]   mem_cache_data_mask,
    //
    output      [31:0]  exe_data_address_next,
    output reg  [31:0]  mem_data_address,
    //
    output reg          exe_branch_start,
    output reg  [31:0]  exe_branch_address,
    //
    output              exe_busy
);

    wire            exe_int_overflow_c;
    wire [31:0]     multi_hi, multi_lo;
    reg  [31:0]     exe_shift_c, exe_result_c;
    wire            exe_load_address_error_c;
    wire            exe_store_address_error_c;
    wire            exe_cache_enabled_c;
//------------------------------------------------------------------------------

    `include "defines.v"
    
    wire    reset_stage = ~exe_clear && ~rst_n;

    always @(posedge clk or negedge reset_stage) begin
        if (~reset_stage) begin
            mem_int_overflow    <= 'b0;
            mem_load_address_error <= 'b0;
            mem_store_address_error <= 'b0;
            mem_cache_enabled       <= 'b0;
        end
        else if (~exe_stall) begin
            mem_int_overflow    <= exe_int_overflow_c;
            mem_load_address_error <= exe_load_address_error_c;
            mem_store_address_error <= exe_store_address_error_c;
            mem_cache_enabled       <= exe_cache_enabled_c;
        end
//        else begin
//            mem_int_overflow    <= 'b0;
//            mem_load_address_error <= 'b0;
//            mem_store_address_error <= 'b0;
//        end
    end
    
//------------------------------------------------------------------------------
    
    assign exe_data_address_next = exe_a + { {16{exe_instant_value[15]}}, exe_instant_value[15:0] };
    
    assign exe_cache_enabled_c = &{config_cache_enabled, (exe_data_address_next[28:12] == 32'h1F800) , (exe_data_address_next[30:29] == 2'b00)} || config_cache_isoloation;
    
    always @(posedge clk or negedge reset_stage) begin
        if(~reset_stage)    mem_data_address <= 32'd0;
        else                mem_data_address <= exe_data_address_next;
    end
    
    assign exe_load_address_error_c = exe_load && (
        ((exe_su_load == `LOAD_SHALF || exe_su_load == `LOAD_UHALF) && exe_data_address_next[0]) ||
        (|{exe_su_load == `LOAD_WORD, exe_su_load == `LOAD_COP2} && exe_data_address_next[1:0] != 2'b00) ||
        (~(config_kernel_mode) && exe_data_address_next[31]));
    
    assign exe_store_address_error_c = exe_store && (
        (exe_su_load == `STORE_HALF && exe_data_address_next[0]) ||
        (|{exe_su_load == `STORE_WORD, exe_su_load == `STORE_COP2} && exe_data_address_next[1:0] != 2'b00) ||
        (~(config_kernel_mode) && exe_data_address_next[31]));
    

//------------------------------------------------------------------------------
    
    always @* begin
        case (exe_su_branch)
            `BRANCH_JUMP   : exe_branch_start <=   1'b1; // Jump and Jump link
            `BRANCH_JUMP_R : exe_branch_start <=   1'b1; // Jump and Jump link 
            `BRANCH_EQ	   : exe_branch_start <=  (exe_a == exe_b); // Branch Equal
            `BRANCH_NEQ    : exe_branch_start <= ~(exe_a == exe_b); // Branch not Equal
            `BRANCH_LEZ	   : exe_branch_start <=  (exe_a[31] ==    'd1) || (exe_a == 'd0); // Branch Less and = to zero
            `BRANCH_GZ	   : exe_branch_start <=  (exe_a[31] ==    'd0) && ~(exe_a == 'd0); // Branch Greater to zero
            `BRANCH_LZ	   : exe_branch_start <=  (exe_a[31] ==    'd1); // JUMP Regature and jump reg and link
            default        : exe_branch_start <=  'b0; // nothing
        endcase
    end
    
    // Do I jump or branch
    
    wire [31:0] instant_number =   {{14{exe_instant_value[15]}}, exe_instant_value[15:0],2'd0};
    
    always @* begin
        case (exe_su_branch)
            `BRANCH_JUMP_R :                                    exe_branch_address <= exe_a; // Jump and Jump lin
            `BRANCH_JUMP   :                                    exe_branch_address <= {exe_pc[31:29], exe_instant_value[26:0],2'd0}; // Jump and Jump line - we stay in the same operation areas
            default        :                                    exe_branch_address <= exe_pc + instant_number; // nothing
        endcase
    end
    
    always @(posedge clk or negedge reset_stage) begin
        if(~reset_stage)                                        mem_branched <= 2'd0;
        else if(exe_branch_start)                               mem_branched <= 2'd1;
        else if(~exe_stall && mem_branched == 2'd1)             mem_branched <= 2'd2;
        else if(exe_stall)                                      mem_branched <= 2'd0;
    end
    
    always @(posedge clk or negedge reset_stage) begin
        if(~reset_stage)                                        mem_branch_address <= 32'd0;
        else if(exe_branch_start)                               mem_branch_address <= exe_branch_address;
    end

//------------------------------------------------------------------------------
// ALU
    
    wire SIGNED_NUMBER_A = exe_su_signed_alu && exe_a[31];
    wire SIGNED_NUMBER_B = exe_su_signed_alu && exe_b[31];
    
    wire [31:0]     exe_add_result = {SIGNED_NUMBER_A, exe_a} + {SIGNED_NUMBER_B, exe_b};
    wire [31:0]     exe_sub_result = {SIGNED_NUMBER_A, exe_a} - {SIGNED_NUMBER_B, exe_b};
     
     always @* begin
        case (exe_su_alu)
            `APU_ADD   : exe_result_c <=    exe_add_result; // Add
            `APU_SUB   : exe_result_c <=    exe_sub_result; // Sub
            `APU_SLT   : exe_result_c <=   ({SIGNED_NUMBER_A, exe_a} < {SIGNED_NUMBER_B, exe_b}) ? 32'h00000001 : 32'h00000000; // Greater than
            `APU_AND   : exe_result_c <=    (exe_a & exe_b); // And
            `APU_OR	   : exe_result_c <=    (exe_a | exe_b); // Or
            `APU_XOR   : exe_result_c <=    (exe_a ^ exe_b); // Xor
            `APU_NOR   : exe_result_c <=   ~(exe_a | exe_b); // Nor
            default    : exe_result_c <=     exe_b; // nothing
        endcase
    end
    
    assign exe_int_overflow_c =
        ((exe_su_alu == `APU_ADD) && (
            (exe_a[31] == 1'b1 && exe_b[31] == 1'b1 && exe_add_result[31] == 1'b0)   ||
            (exe_a[31] == 1'b0 && exe_b[31] == 1'b0 && exe_add_result[31] == 1'b1))) ||
        (exe_su_alu == `APU_SUB && (
            (exe_a[31] == 1'b1 && exe_b[31] == 1'b0 && exe_sub_result[31] == 1'b0)   ||
            (exe_a[31] == 1'b0 && exe_b[31] == 1'b1 && exe_sub_result[31] == 1'b1)));

//------------------------------------------------------------------------------ shift
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Shift ALU
    
    wire [4:0] shifting_amount = exe_su_shift_in ? exe_instant_value[10:6] : exe_a[4:0];
    
    wire [63:0] shift_right_signed = { {32{exe_b[31]}}, exe_b};
    
    wire [63:0] shift_right_result = shift_right_signed >> shifting_amount;
    
    always @* begin
        casez (exe_su_shift)
            `SU_SHIFT_SLL	  : exe_shift_c <= exe_b <<< shifting_amount;
            `SU_SHIFT_SRL     : exe_shift_c <= exe_b >> shifting_amount;
            default           : exe_shift_c <= shift_right_result[31:0];
            // Special Area
        endcase
    end

//------------------------------------------------------------------------------
    
    always @(posedge clk or negedge reset_stage) begin
        if (~reset_stage) begin
            mem_result <= 'b0;
        end
        else begin
            case (exe_su_result_mux)
                `SU_RESULT_INST     : mem_result <= {{15{exe_instant_value[15]}}, exe_instant_value[15:0]};
                `SU_RESULT_ALU      : mem_result <= exe_result_c;
                `SU_RESULT_SHIFT    : mem_result <= exe_shift_c;
                `SU_RESULT_HI       : mem_result <= multi_hi;
                `SU_RESULT_LO       : mem_result <= multi_lo;
                default             : mem_result <= exe_b;
            endcase
        end
    end

//------------------------------------------------------------------------------
    
    block_muldiv block_muldiv_inst(
        .clk                (clk),
        .rst_n              (rst_n),
        
        .rf_mthi            (exe_mthi),
        .rf_mtlo            (exe_mtlo),
        .rf_mult            (exe_mult),
        .rf_multu           (exe_multu),
        .rf_div             (exe_div),
        .rf_divu            (exe_divu),
        
        .exe_a              (exe_a),                //input [31:0]
        .exe_b              (exe_b),                //input [31:0]
        
        .muldiv_busy        (exe_busy),          //output
        
        .multi_lo           (multi_lo),
        .multi_hi           (multi_hi)
    );
    
/*

    cache controller / scrachpad

*/


    always @* begin
        casez ({exe_cache_enabled_c, exe_su_store, exe_data_address_next[1:0]})
            {1'b1, `STORE_BYTE, 2'b11} :   mem_cache_data_mask <=    4'b0001;
            {1'b1, `STORE_BYTE, 2'b10} :   mem_cache_data_mask <=    4'b0010;
            {1'b1, `STORE_BYTE, 2'b01} :   mem_cache_data_mask <=    4'b0100;
            {1'b1, `STORE_BYTE, 2'b00} :   mem_cache_data_mask <=    4'b1000;
            
            {1'b1, `STORE_HALF, 2'b00} :   mem_cache_data_mask <=    4'b1100;
            {1'b1, `STORE_HALF, 2'b10} :   mem_cache_data_mask <=    4'b0011;
            
            {1'b1, `STORE_COP2, 2'b00} ,
            {1'b1, `STORE_WORD, 2'b00} :   mem_cache_data_mask <=    4'b1111;
            
            {1'b1, `STORE_SWR, 2'b11}  :   mem_cache_data_mask <=    4'b1000;
            {1'b1, `STORE_SWR, 2'b10}  :   mem_cache_data_mask <=    4'b1100;
            {1'b1, `STORE_SWR, 2'b01}  :   mem_cache_data_mask <=    4'b1110;
            {1'b1, `STORE_SWR, 2'b00}  :   mem_cache_data_mask <=    4'b1111;
            
            {1'b1, `STORE_SWL, 2'b11}  :   mem_cache_data_mask <=    4'b0001;
            {1'b1, `STORE_SWL, 2'b10}  :   mem_cache_data_mask <=    4'b0011;
            {1'b1, `STORE_SWL, 2'b01}  :   mem_cache_data_mask <=    4'b0111;
            {1'b1, `STORE_SWL, 2'b00}  :   mem_cache_data_mask <=    4'b1111;
            
            default                    :   mem_cache_data_mask <=    4'b0000;
        endcase
    end

endmodule
