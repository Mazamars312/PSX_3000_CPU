`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.07.2020 07:23:26
// Design Name: 
// Module Name: aor3000_pipeline
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module aor3000_pipeline(

    input                   clk,
    input                   rst_n,
    
    input                   config_kernel_mode, // this is for the load and store error checking on the EXE stage
    input                   config_cache_enabled, // On store to the scrapad this will happen on the EXE stage
    input                   config_isolate_cache,
    
    // RF Output
    
    input       [4:0]       rf_address_a,       // Reg Address access for the RS/A side
    input       [4:0]       rf_address_b,       // Reg Address access for the RT/B side
    input                   rf_stall,           // RF stage stall - Off is the pipeline will move
    
    
    input       [1:0]       exe_a_bypass,       // this is the bypass on the exe stage for the RF result to the ALU stage
    input       [2:0]       exe_b_bypass,       // this is the bypass on the exe stage or storeage for the RF result to the ALU stage
    
    output      [31:0]      exe_b,               // the exe_b RT is used for storing to the cache/scrachpad
    
    // EXE stage core
    input                   exe_stall,          // stall the EXE stage
    input                   exe_clear,          // clear the EXE Stage
    
    input       [26:0]      exe_instant_value,  // Instant for Jumps and ALU calulations

    input       [31:0]      exe_pc,             // This is used for Jumps and the calculations

    input       [3:0]       exe_su_alu,         // selecter for the ALU stage exe
    input       [2:0]       exe_su_branch,      // selecter for the branch stage exe
    input       [4:0]       exe_su_load,        // selecter for the load stage exe for checking
    input       [4:0]       exe_su_store,       // selecter for the storing stage exe for checking
    input       [2:0]       exe_su_shift,       // selecter for the shifter in the EXE stage
    input       [3:0]       exe_su_result_mux,  // selecter for the result output of the EXE Stage
    input                   exe_su_shift_in,    // This selects the shifter for instant value to be used for the shift
    input                   exe_su_signed_alu,  // Selecter for signed numbers

    input                   exe_mthi,           // this stores to the HI reg
    input                   exe_mtlo,           // this stores to the LO reg
    input                   exe_mult,           // this does a multi signed in the EXE stage
    input                   exe_multu,          // this does a multi unsigned in the EXE stage
    input                   exe_div,            // this does a div signed in the EXE stage
    input                   exe_divu,           // this does a div unsigned in the EXE stage
    //
    input                   exe_load,           // this does a signal for a load command in the EXE stage
    input                   exe_store,          // this does a signal for a store command in the EXE stage
    output  [31:0]          exe_data_address_next, // this is used for the cached address on both load and stores
        
    output                  exe_branch_start,       // this does a signal for a branch/jump true to the PC counter
    output  [31:0]          exe_branch_address,     // this does a signal for a branch/jump address to the PC counter
        //
    output                  exe_busy,           // this tells us that the DIV/Multi is still running - 0cycle

    // MEM Write back
    
    output      [31:0]      mem_result,         // this is not really used. Will think about this

    output                  mem_load_address_error, // This shows the controller that there is a load issue. Exception in the WB stage needs to happen
    output                  mem_store_address_error,// This shows the controller that there is a store issue. Exception in the WB stage needs to happen

    output                  mem_int_overflow,       // This shows the controller that there is a overflow issue. Exception in the WB stage needs to happen
    output      [3:0]       mem_cache_data_mask,    // cache store mask.
    output                  mem_cache_enabled,      // this is an address decode to the controller saying that this load or store is to the cache system
    
    output      [1:0]       mem_branched,           // this is for the COP2 to see if a exception happens on the 
    output      [31:0]      mem_branch_address,     // Address of the branched locations
    output      [31:0]      mem_data_address,       // the address for data locations

    input                   mem_stall,              // to stall the MEM stage of the core
    input                   mem_clear,              // to clear the MEM stage of the core
    input                   mem_little_endian,      // This only changes the masking for externally, the data is changed on the interface location as in the core we will always run bigendin
    
    input                   mem_store_enable,       //  this is for the MEM store out to the fifo core
   
    input       [4:0]       mem_su_load,            // selector for the load commands
    input       [4:0]       mem_su_store,           // selector for the Store commands
    
    input       [31:0]      mem_data_cache_input,   // This is the cache output to the MEM data stage
    input       [31:0]      mem_data_COP0_input,    // This is the COP0 output to the MEM data stage
    input       [31:0]      mem_data_COP2_input,    // This is the COP2 output to the MEM data stage
    output      [31:0]      mem_cache_io_reg,       // This is the cache IO reg output from the mem stage for cache control
    //
    output      [68:0]      mem_external_fifo_data, // [68] Write =1, [67:64] Writemask , [63:32] address , [31:0] Data to fifo controller
    input       [31:0]      mem_external_result,    // the input from the external for the CPU
    
    // WB Stage
    
    input       [4:0]       wb_result_index         // this is for the writeback to the regs
    
    );
    
    // RF stage 
    
    wire  [31:0]      exe_a;
    
    
    // WB Stage
    

    wire  [31:0]      wb_result;
    
    
    pipeline_rf pipeline_rf(
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        .rf_stall               (rf_stall),
        
        .exe_instant_value      (exe_instant_value),
        
        .exe_a_bypass           (exe_a_bypass),
        .exe_b_bypass           (exe_b_bypass),
        
        // MEM Bypass core
        .mem_result             (mem_result),
        // WB Write back
        .wb_result_index        (wb_result_index),
        .wb_result              (wb_result),
        // RF Output
        .rf_address_a           (rf_address_a),
        .rf_address_b           (rf_address_b),
        .exe_a                  (exe_a),
        .exe_b                  (exe_b)
    );
    
    pipeline_exe pipeline_exe(
        .clk                        (clk),
        .rst_n                      (rst_n),
        //
        .exe_stall                  (exe_stall),
        .exe_clear                  (exe_clear),
        
        .config_kernel_mode         (config_kernel_mode),
        .config_cache_enabled       (config_cache_enabled),
        .config_cache_isoloation    (config_isolate_cache),
    
        .exe_a                      (exe_a),
        .exe_b                      (exe_b),
        
        .exe_pc                     (exe_pc),
        
        .exe_instant_value          (exe_instant_value),
    
        .exe_su_alu                 (exe_su_alu),
        .exe_su_branch              (exe_su_branch),
        .exe_su_load                (exe_su_load),
        .exe_su_store               (exe_su_store),
        .exe_su_shift               (exe_su_shift),
        .exe_su_result_mux          (exe_su_result_mux),
        .exe_su_shift_in            (exe_su_shift_in),
        .exe_su_signed_alu          (exe_su_signed_alu),
    
        .exe_mthi                   (exe_mthi),
        .exe_mtlo                   (exe_mtlo),
        .exe_mult                   (exe_mult),
        .exe_multu                  (exe_multu),
        .exe_div                    (exe_div),
        .exe_divu                   (exe_divu),
        //
        .exe_load                   (exe_load),
        .exe_store                  (exe_store),
        .exe_data_address_next      (exe_data_address_next),
        
        .exe_branch_start           (exe_branch_start),
        .exe_branch_address         (exe_branch_address),
        //
        .exe_busy                   (exe_busy),
        //
        .mem_branched               (mem_branched),
        .mem_branch_address         (mem_branch_address),
        .mem_cache_data_mask        (mem_cache_data_mask),
        .mem_cache_enabled          (mem_cache_enabled),
        .mem_load_address_error     (mem_load_address_error),
        .mem_store_address_error    (mem_store_address_error),
    
        .mem_int_overflow           (mem_int_overflow),
        //
        .mem_result                 (mem_result),
        //
        .mem_data_address           (mem_data_address)
        //
    );
    
    pipeline_mem pipeline_mem(
        .clk                        (clk),
        .rst_n                      (rst_n),
        //
        .mem_stall                  (mem_stall),
        .mem_clear                  (mem_clear),
        .mem_little_endian          (mem_little_endian),
        
        .mem_store_enable           (mem_store_enable),
       
        .mem_su_load                (mem_su_load),
        .mem_su_store               (mem_su_store),
        //
        .mem_result                 (mem_result),
        //
        .mem_data_address           (mem_data_address),
        //
        .mem_data_cache_input       (mem_data_cache_input),
        .mem_data_COP0_input        (mem_data_COP0_input),
        .mem_data_COP2_input        (mem_data_COP2_input),
        .mem_cache_io_reg           (mem_cache_io_reg),
        //
        .mem_external_fifo_data     (mem_external_fifo_data), // [68] Write =1, [67:64] Writemask , [63:32] address , [31:0] Data
        .mem_external_result        (mem_external_result),
        //
        .wb_result                  (wb_result)
    );
    
    
endmodule
