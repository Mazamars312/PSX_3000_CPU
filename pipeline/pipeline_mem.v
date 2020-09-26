/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module pipeline_mem(
    input               clk,
    input               rst_n,
    //
    input               mem_stall,
    input               mem_clear,
    input               mem_little_endian,
    
    input               mem_store_enable,
   
    input       [4:0]   mem_su_load,
    input       [4:0]   mem_su_store,
    //
    input       [31:0]  mem_result,
    //
    input       [31:0]  mem_data_address,
    //
    input       [31:0]  mem_data_cache_input,
    input       [31:0]  mem_data_COP0_input,
    input       [31:0]  mem_data_COP2_input,
    output reg  [31:0]  mem_cache_io_reg,
    //
    output      [68:0]  mem_external_fifo_data, // [68] Write =1, [67:64] Writemask , [63:32] address , [31:0] Data
    input       [31:0]  mem_external_result,
    //
    output reg  [31:0]  wb_result
);

    `include "defines.v"

    wire reset_stage = rst_n || ~mem_clear;
    
    reg [31:0]      wb_result_c;
    reg [31:0]      mem_aligment_c;
    reg [31:0]      mem_location_c;
    reg [31:0]      left_align_load_c;
    reg [31:0]      right_align_load_c;
    reg [31:0]      left_align_store_c;
    reg [31:0]      right_align_store_c;
    reg [31:0]      store_result_c;
    reg [3:0]       mem_data_mask_c;
    
    wire cache_io_port = mem_data_address == 32'hFFFE0130;
    wire cache_memory = (mem_data_address[28:12] == 32'h1F800) && (mem_data_address[30:29] == 2'b00);

    always @(posedge clk or negedge reset_stage) begin
        if (~reset_stage) begin
//            mem_external_fifo_data  <= 'b0;
            wb_result               <= 'b0;
        end
        else if (~mem_stall) begin
//            mem_external_fifo_data  <= {mem_store_enable, mem_data_mask_c, mem_data_address, store_result_c};
            wb_result               <= wb_result_c;
        end
    end
    
    assign mem_external_fifo_data  = {mem_store_enable, mem_data_mask_c, mem_data_address, store_result_c};


    always @(posedge clk or negedge rst_n) begin
        if (~rst_n)                                 mem_cache_io_reg <= 'b0;
        else if (cache_io_port && mem_store_enable) mem_cache_io_reg <= mem_result;
    end

/*
    LOAD Cores

    `define LOAD_UBYTE      4'd01
    `define LOAD_SBYTE      4'd02
    `define LOAD_UHALF      4'd03
    `define LOAD_SHALF      4'd04
    `define LOAD_WORD       4'd05
    `define LOAD_LWR        4'd06
    `define LOAD_LWL        4'd07
    `define LOAD_COP0       4'd08
    `define LOAD_COP2       4'd09
*/


// Here is the data cache/scrachpad and the cache port IO or external data
    always @* begin
        casez ({cache_memory, cache_io_port})
            2'b1z   :   mem_location_c <= mem_data_cache_input;
            2'b01   :   mem_location_c <= mem_cache_io_reg;
            default :   mem_location_c <= mem_external_result;
        endcase
    end
// here is the shifter for the normal addresses
    always @* begin
        case(mem_data_address[1:0])
            2'b11   :   mem_aligment_c  <= {mem_location_c[23:0], mem_location_c[31:24]};
            2'b10   :   mem_aligment_c  <= {mem_location_c[15:0], mem_location_c[31:16]};
            2'b01   :   mem_aligment_c  <= {mem_location_c[7:0],  mem_location_c[31:8]};
            default :   mem_aligment_c  <=  mem_location_c;
        endcase
    end

// Left shifter 
    always @* begin
        case(mem_data_address[1:0])
            2'b11   :   left_align_load_c  <= {mem_result[31:8], mem_location_c[31:24]};
            2'b10   :   left_align_load_c  <= {mem_result[31:16], mem_location_c[31:16]};
            2'b01   :   left_align_load_c  <= {mem_result[31:24],  mem_location_c[31:8]};
            default :   left_align_load_c  <=  mem_location_c;
        endcase
    end

// Right shifter   
    always @* begin
        case(mem_data_address[1:0])
            2'b11   :   right_align_load_c  <=  mem_location_c;
            2'b10   :   right_align_load_c  <= {mem_location_c[23:0],    mem_result[7:0]};
            2'b01   :   right_align_load_c  <= {mem_location_c[15:0],    mem_result[15:0]};
            default :   right_align_load_c  <= {mem_location_c[7:0],     mem_result[23:0]};
        endcase
    end

// Load shifter

    always @* begin
        case (mem_su_load)
            `LOAD_UBYTE :   wb_result_c <= { 24'd0,                     mem_aligment_c[7:0]};
            `LOAD_SBYTE :   wb_result_c <= {{24{mem_aligment_c[7]}},    mem_aligment_c[7:0]};
            `LOAD_UHALF :   wb_result_c <= { 16'd0,                     mem_aligment_c[15:0]};
            `LOAD_SHALF :   wb_result_c <= {{16{mem_aligment_c[15]}},   mem_aligment_c[15:0]};
            `LOAD_WORD  :   wb_result_c <=      mem_aligment_c;
            `LOAD_LWR   :   wb_result_c <=      right_align_load_c;
            `LOAD_LWL   :   wb_result_c <=      left_align_load_c;
            `LOAD_COP0  :   wb_result_c <=      mem_data_COP0_input;
            `LOAD_COP2  :   wb_result_c <=      mem_data_COP2_input;
            default     :   wb_result_c <=      mem_result;
        endcase
    end
    
    // Store shifter
    
    /*
    
    `define STORE_BYTE      3'd01
    `define STORE_HALF      3'd02
    `define STORE_WORD      3'd03
    `define STORE_SWR       3'd04
    `define STORE_SWL       3'd05
    `define STORE_COP2      3'd06
    
    */
    
    always @* begin
        case (mem_su_store)
            `STORE_BYTE :   store_result_c <= {4{mem_result[7:0]}};
            `STORE_HALF :   store_result_c <= {2{mem_result[15:0]}};
            `STORE_COP2 :   store_result_c <=    mem_data_COP2_input;
            default     :   store_result_c <=    mem_result;
        endcase
    end
    
    always @* begin
        casez ({mem_little_endian, mem_su_store,mem_data_address[1:0]})
            {1'b0, `STORE_BYTE, 2'b11} :   mem_data_mask_c <=    4'b0001;
            {1'b0, `STORE_BYTE, 2'b10} :   mem_data_mask_c <=    4'b0010;
            {1'b0, `STORE_BYTE, 2'b01} :   mem_data_mask_c <=    4'b0100;
            {1'b0, `STORE_BYTE, 2'b00} :   mem_data_mask_c <=    4'b1000;
            
            {1'b0, `STORE_HALF, 2'b00} :   mem_data_mask_c <=    4'b1100;
            {1'b0, `STORE_HALF, 2'b10} :   mem_data_mask_c <=    4'b0011;
            
            {1'b0, `STORE_COP2, 2'b00} ,
            {1'b0, `STORE_WORD, 2'b00} :   mem_data_mask_c <=    4'b1111;
            
            {1'b0, `STORE_SWR, 2'b11}  :   mem_data_mask_c <=    4'b1000;
            {1'b0, `STORE_SWR, 2'b10}  :   mem_data_mask_c <=    4'b1100;
            {1'b0, `STORE_SWR, 2'b01}  :   mem_data_mask_c <=    4'b1110;
            {1'b0, `STORE_SWR, 2'b00}  :   mem_data_mask_c <=    4'b1111;
            
            {1'b0, `STORE_SWL, 2'b11}  :   mem_data_mask_c <=    4'b0001;
            {1'b0, `STORE_SWL, 2'b10}  :   mem_data_mask_c <=    4'b0011;
            {1'b0, `STORE_SWL, 2'b01}  :   mem_data_mask_c <=    4'b0111;
            {1'b0, `STORE_SWL, 2'b00}  :   mem_data_mask_c <=    4'b1111;
            
            {1'b1, `STORE_BYTE, 2'b11} :   mem_data_mask_c <=    4'b1000;
            {1'b1, `STORE_BYTE, 2'b10} :   mem_data_mask_c <=    4'b0100;
            {1'b1, `STORE_BYTE, 2'b01} :   mem_data_mask_c <=    4'b0010;
            {1'b1, `STORE_BYTE, 2'b00} :   mem_data_mask_c <=    4'b0001;
            
            {1'b1, `STORE_HALF, 2'b00} :   mem_data_mask_c <=    4'b0011;
            {1'b1, `STORE_HALF, 2'b10} :   mem_data_mask_c <=    4'b1100;
            
            {1'b1, `STORE_COP2, 2'b00} ,
            {1'b1, `STORE_WORD, 2'b00} :   mem_data_mask_c <=    4'b1111;
            
            {1'b1, `STORE_SWR, 2'b11}  :   mem_data_mask_c <=    4'b0001;
            {1'b1, `STORE_SWR, 2'b10}  :   mem_data_mask_c <=    4'b0011;
            {1'b1, `STORE_SWR, 2'b01}  :   mem_data_mask_c <=    4'b0111;
            {1'b1, `STORE_SWR, 2'b00}  :   mem_data_mask_c <=    4'b1111;
            
            {1'b1, `STORE_SWL, 2'b11}  :   mem_data_mask_c <=    4'b1000;
            {1'b1, `STORE_SWL, 2'b10}  :   mem_data_mask_c <=    4'b1100;
            {1'b1, `STORE_SWL, 2'b01}  :   mem_data_mask_c <=    4'b1110;
            {1'b1, `STORE_SWL, 2'b00}  :   mem_data_mask_c <=    4'b1111;
            default                    :   mem_data_mask_c <=    4'b0000;
        endcase
    end

endmodule
