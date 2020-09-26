/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module psx_cpu_3000(
    input               clk,
    input               rst_n,
    
    //
    input       [5:0]   i_interrupt_vector,
    
    //
    output      [31:0]  o_avm_address,
    output      [31:0]  o_avm_writedata,
    output      [3:0]   o_avm_byteenable,
    output      [2:0]   o_avm_burstcount,
    output              o_avm_write,
    output              o_avm_read,
    
    input               i_avm_waitrequest,
    input               i_avm_readdatavalid,
    input       [31:0]  i_avm_readdata
);

//------------------------------------------------------------------------------

wire    [31:0]      PC_counter;
wire    [31:0]      Branch_address;
wire    [31:0]      exception_start_pc;
wire                PC_update;
wire                Branch_update;
wire                pc_kill_instuction;

wire    [31:0]      rf_instuction;

wire    [4:0]       rf_address_a;       // Reg Address access for the RS/A side
wire    [4:0]       rf_address_b;       // Reg Address access for the RT/B side
wire                rf_stall;           // RF stage stall - Off is the pipeline will move
wire    [31:0]      RF_counter;

wire    [1:0]       exe_a_bypass;       // this is the bypass on the exe stage for the RF result to the ALU stage
wire    [2:0]       exe_b_bypass;       // this is the bypass on the exe stage or storeage for the RF result to the ALU stage

wire    [31:0]      exe_pc;

wire    [26:0]      exe_instant_value;

wire    [31:0]      exe_b;

wire    [3:0]       exe_su_alu;
wire    [2:0]       exe_su_branch;
wire    [4:0]       exe_su_load;
wire    [4:0]       exe_su_store;
wire    [2:0]       exe_su_shift;
wire    [3:0]       exe_su_result_mux;
wire                exe_su_shift_in;
wire                exe_su_signed_alu;

wire                exe_mthi;
wire                exe_mtlo;
wire                exe_mult;
wire                exe_multu;
wire                exe_div;
wire                exe_divu;
//
wire                exe_load;
wire                exe_store;

wire    [31:0]      mem_data_cache_input;
wire    [31:0]      external_address_out;
wire    [4:0]       mem_su_load;
wire    [4:0]       mem_su_store;

wire    [31:0]      mem_data_address;


wire    [31:0]      mem_result;
wire    [31:0]      mem_data_COP0_input;
wire    [31:0]      mem_branch_address;
wire    [1:0]       mem_branched;
wire    [31:0]      mem_pc;
wire    [31:0]      mem_instr;

wire    [68:0]      mem_external_fifo_data;
wire    [68:0]      ram_fifo_q;
wire    [31:0]      ram_result;
wire    [31:0]      ram_instr_address;  

wire    [4:0]       mem_cache_data_mask;

wire    [4:0]       wb_result_index;

`define simulation_rom_if

`ifdef simulation_rom_if
    
    reg [31:0]  if_rom  [131000:0];
    reg         pass;
    reg [31:0] if_rom_external;

    initial begin
        $readmemh("SONY_BIOS1002.MEM", if_rom);
        pass = 0;
    end
    always @(posedge clk)    pass <= PC_update;
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n)if_rom_external <= 'b0;
        else if_rom_external <= if_rom[PC_counter[11:2]];
    end
//`else 


`endif



aor3000_pc_core aor3000_pc_core(
    .clk                        (clk),
    .rst_n                      (rst_n),
    
    .i_PC_update                (PC_update),
    .i_Branch_address           (Branch_address),
    .i_Branch_update            (Branch_update),
    .i_Exception_address        (exception_start_pc),
    .i_Exception_update         (exception_start),
    
    .o_PC_counter               (PC_counter),  
    .o_RF_counter               (RF_counter),
    .o_pc_kill_instuction       (pc_kill_instuction)  
    );

//GTEEngine GTEEngine (
//    .i_clk          (clk),
//    .i_nRst         (rst_n),

//    //   GTE PORT
//    .i_regID        (i_regID),
//    .i_WritReg      (i_WritReg),
//    .i_ReadReg      (i_ReadReg),
//    .i_dataIn       (i_dataIn),
//    .o_dataOut      (o_dataOut),

//    .i_Instruction  (i_Instruction),
//    .i_run          (i_run),
//    .o_executing    (o_executing)    // SET TO ZERO AT LAST CYCLE OF EXECUTION !!!! Shave off a cycle.
//);

//------------------------------------------------------------------------------

aor3000_block_cp0 aor3000_block_cp0(
    .clk                        (clk),
    .rst_n                      (rst_n),
    //
    .interrupt_vector           (i_interrupt_vector),
    
    //
    .config_switch_caches       (config_switch_caches),
    .config_isolate_cache       (config_isolate_cache),
    .config_coproc0_usable      (config_coproc0_usable),
    .config_coproc1_usable      (config_coproc1_usable),
    .config_coproc2_usable      (config_coproc2_usable),
    .config_kernel_mode         (config_kernel_mode),
    //
    .exe_instr                  (exe_instant_value),
    .exe_b                      (exe_b),
    .exe_mtc0                   (exe_mtc0),
    .exe_rfe                    (exe_rfe),
    //
    .mem_store_address_error    (mem_store_address_error),
    .mem_load_address_error     (mem_load_address_error),
    .mem_syscall                (mem_syscall), 
    .mem_break                  (mem_break), 
    .mem_reserved_instr         (mem_reserved_instr), 
    .mem_coproc_unusable        (mem_coproc_unusable), 
    .mem_int_overflow           (mem_int_overflow),
    .mem_little_endian          (mem_little_endian),
    
    .mem_coproc0_output         (mem_data_COP0_input),
 
    //
    .sr_cm_set                  (sr_cm_set),
    .sr_cm_clear                (sr_cm_clear),

    //
    .exception_start            (exception_start),
    .exception_start_pc         (exception_start_pc),
    
    //
    .mem_stall                  (mem_stall),
    .mem_instr                  (mem_instr),
    .mem_pc                     (mem_pc),
    .mem_branched               (mem_branched),
    .mem_branch_address         (mem_branch_address),
    .mem_address                (mem_data_address)
    
); 

aor3000_contoller aor3000_contoller(
    .clk                        (clk),
    .rst_n                      (rst_n),
    .config_kernel_mode         (config_kernel_mode), // this is for the load and store error checking on the EXE stage
    .config_cache_enabled       (config_cache_enabled), // On store to the scrapad this will happen on the EXE stage
    .config_isolate_cache       (config_isolate_cache),
    .config_coproc0_usable      (config_coproc0_usable),
    .config_coproc1_usable      (config_coproc1_usable),
    .config_coproc2_usable      (config_coproc2_usable),
    // The RF stage
    `ifdef simulation_rom_if
    .rf_instruction             (if_rom_external),                  // Output switch for the instruction, the External access or Cache mem goes througha mux for this
    .rf_valid                   (pass),
    `else 
    .rf_instruction             (rf_instuction),                  // Output switch for the instruction, the External access or Cache mem goes througha mux for this
    .rf_valid                   (rf_valid),
    `endif
    .rf_pc_update               (PC_update),
                           // This is for HIT's and or external update and next cycle access
    .rf_pc_location             (RF_counter),
    
    .i_pc_kill_instuction       (pc_kill_instuction) ,
    // RF Output
    
    .rf_address_a               (rf_address_a),       // Reg Address access for the RS/A side
    .rf_address_b               (rf_address_b),       // Reg Address access for the RT/B side
    .rf_stall                   (rf_stall),           // RF stage stall - Off is the pipeline will move
    
    // EXE stage core
    .exe_a_bypass               (exe_a_bypass),    // this is the bypass on the exe stage for the RF result to the ALU stage
    .exe_b_bypass               (exe_b_bypass),    // this is the bypass on the exe stage or storeage for the RF result to the ALU stage
    
    .exe_stall                  (exe_stall),          // stall the EXE stage
    .exe_clear                  (exe_clear),          // clear the EXE Stage
    
    .exe_instant_value          (exe_instant_value),  // Instant for Jumps and ALU calulations

    .exe_pc                     (exe_pc),             // This is used for Jumps and the calculations

    .exe_su_alu                 (exe_su_alu),         // selecter for the ALU stage exe
    .exe_su_branch              (exe_su_branch),      // selecter for the branch stage exe
    .exe_su_load                (exe_su_load),        // selecter for the load stage exe for checking
    .exe_su_store               (exe_su_store),       // selecter for the storing stage exe for checking
    .exe_su_shift               (exe_su_shift),       // selecter for the shifter in the EXE stage
    .exe_su_result_mux          (exe_su_result_mux),  // selecter for the result output of the EXE Stage
    .exe_su_shift_in            (exe_su_shift_in),    // This selects the shifter for instant value to be used for the shift
    .exe_su_signed_alu          (exe_su_signed_alu),  // Selecter for signed numbers

    .exe_mthi                   (exe_mthi),           // this stores to the HI reg
    .exe_mtlo                   (exe_mtlo),           // this stores to the LO reg
    .exe_mult                   (exe_mult),           // this does a multi signed in the EXE stage
    .exe_multu                  (exe_multu),          // this does a multi unsigned in the EXE stage
    .exe_div                    (exe_div),            // this does a div signed in the EXE stage
    .exe_divu                   (exe_divu),           // this does a div unsigned in the EXE stage
    //
    .exe_load                   (exe_load),           // this does a signal for a load command in the EXE stage
    .exe_store                  (exe_store),          // this does a signal for a store command in the EXE stage

    .exe_busy                   (exe_busy),
    .exe_COP2_busy              (exe_COP2_busy),
    
    .exe_mtc0                   (exe_mtc0),
    .exe_rfe                    (exe_rfe),
    // MEM Write back


    .mem_stall                  (mem_stall),              // to stall the MEM stage of the core
    .mem_clear                  (mem_clear),              // to clear the MEM stage of the core
    .mem_reserved_instr         (mem_reserved_instr), 
    .mem_syscall                (mem_syscall), 
    .mem_break                  (mem_break), 
    .mem_coproc_unusable        (mem_coproc_unusable),
    .mem_pc                     (mem_pc), 
    .mem_instr                  (mem_instr),

    .mem_store_enable           (mem_store_enable),       //  this is for the MEM store out to the fifo core
   
    .mem_su_load                (mem_su_load),            // selector for the load commands
    .mem_su_store               (mem_su_store),           // selector for the Store commands
    .mem_ram_fifo_full          (ram_fifo_full),
    .mem_ram_fifo_write         (ram_fifo_wrreq),
    .mem_instrution_result_valid(ram_result_is_read_instr),
    .mem_result_valid           (ram_result_valid),
    .mem_cache_enabled          (mem_cache_enabled),      // this is an address decode to the controller saying that this load or store is to the cache system
    .mem_exception_start        (exception_start),
 
    //WB Stage   
    .wb_result_index            (wb_result_index)         // this is for the writeback to the regs
);



aor3000_pipeline aor3000_pipeline(
    .clk                        (clk),
    .rst_n                      (rst_n),
    .config_kernel_mode         (config_kernel_mode), // this is for the load and store error checking on the EXE stage
    .config_cache_enabled       (config_cache_enabled), // On store to the scrapad this will happen on the EXE stage
    .config_isolate_cache       (config_isolate_cache),
    // RF Output
    
    .rf_address_a               (rf_address_a),       // Reg Address access for the RS/A side
    .rf_address_b               (rf_address_b),       // Reg Address access for the RT/B side
    .rf_stall                   (rf_stall),           // RF stage stall - Off is the pipeline will move
    
    
    .exe_a_bypass               (exe_a_bypass),    // this is the bypass on the exe stage for the RF result to the ALU stage
    .exe_b_bypass               (exe_b_bypass),    // this is the bypass on the exe stage or storeage for the RF result to the ALU stage
    
    .exe_b                      (exe_b),               // the exe_b RT is used for storing to the cache/scrachpad
    
    // EXE stage core
    .exe_stall                  (exe_stall),          // stall the EXE stage
    .exe_clear                  (exe_clear),          // clear the EXE Stage
    
    .exe_instant_value          (exe_instant_value),  // Instant for Jumps and ALU calulations

    .exe_pc                     (exe_pc),             // This is used for Jumps and the calculations

    .exe_su_alu                 (exe_su_alu),         // selecter for the ALU stage exe
    .exe_su_branch              (exe_su_branch),      // selecter for the branch stage exe
    .exe_su_load                (exe_su_load),        // selecter for the load stage exe for checking
    .exe_su_store               (exe_su_store),       // selecter for the storing stage exe for checking
    .exe_su_shift               (exe_su_shift),       // selecter for the shifter in the EXE stage
    .exe_su_result_mux          (exe_su_result_mux),  // selecter for the result output of the EXE Stage
    .exe_su_shift_in            (exe_su_shift_in),    // This selects the shifter for instant value to be used for the shift
    .exe_su_signed_alu          (exe_su_signed_alu),  // Selecter for signed numbers

    .exe_mthi                   (exe_mthi),           // this stores to the HI reg
    .exe_mtlo                   (exe_mtlo),           // this stores to the LO reg
    .exe_mult                   (exe_mult),           // this does a multi signed in the EXE stage
    .exe_multu                  (exe_multu),          // this does a multi unsigned in the EXE stage
    .exe_div                    (exe_div),            // this does a div signed in the EXE stage
    .exe_divu                   (exe_divu),           // this does a div unsigned in the EXE stage
    //
    .exe_load                   (exe_load),           // this does a signal for a load command in the EXE stage
    .exe_store                  (exe_store),          // this does a signal for a store command in the EXE stage
    .exe_data_address_next      (exe_data_address_next), // this is used for the cached address on both load and stores
        
    .exe_branch_start           (Branch_update),       // this does a signal for a branch/jump true to the PC counter
    .exe_branch_address         (Branch_address),     // this does a signal for a branch/jump address to the PC counter
        //
    .exe_busy                   (exe_busy),           // this tells us that the DIV/Multi is still running - 0cycle

    // MEM Write back
    
    .mem_result                 (mem_result),         // this is not really used. Will think about this

    .mem_load_address_error     (mem_load_address_error), // This shows the controller that there is a load issue. Exception in the WB stage needs to happen
    .mem_store_address_error    (mem_store_address_error),// This shows the controller that there is a store issue. Exception in the WB stage needs to happen

    .mem_int_overflow           (mem_int_overflow),       // This shows the controller that there is a overflow issue. Exception in the WB stage needs to happen
    .mem_cache_data_mask        (mem_cache_data_mask),    // cache store mask.
    .mem_cache_enabled          (mem_cache_enabled),      // this is an address decode to the controller saying that this load or store is to the cache system
    
    .mem_branched               (mem_branched),           // this is for the COP2 to see if a exception happens on the 
    .mem_branch_address         (mem_branch_address),     // Address of the branched locations
    .mem_data_address           (mem_data_address),       // the address for data locations

    .mem_stall                  (mem_stall),              // to stall the MEM stage of the core
    .mem_clear                  (mem_clear),              // to clear the MEM stage of the core
    .mem_little_endian          (mem_little_endian),      // This only changes the masking for externally, the data is changed on the interface location as in the core we will always run bigendin
    
    .mem_store_enable           (mem_store_enable),       //  this is for the MEM store out to the fifo core
   
    .mem_su_load                (mem_su_load),            // selector for the load commands
    .mem_su_store               (mem_su_store),           // selector for the Store commands
    
    .mem_data_cache_input       (mem_data_cache_input),   // This is the cache output to the MEM data stage
    .mem_data_COP0_input        (mem_data_COP0_input),    // This is the COP0 output to the MEM data stage
    .mem_data_COP2_input        (mem_data_COP2_input),    // This is the COP2 output to the MEM data stage
    .mem_cache_io_reg           (mem_cache_io_reg),       // This is the cache IO reg output from the mem stage for cache control
    //
    .mem_external_fifo_data     (mem_external_fifo_data),   // [68] Write =1, [67:64] Writemask , [63:32] address , [31:0] Data to fifo controller
    .mem_external_result        (ram_result),               // the input from the external for the CPU
    
    // WB Stage
    
    .wb_result_index            (wb_result_index)         // this is for the writeback to the regs
);



memory_ram memory_ram_1(
    .clk                                (clk),
    .rst_n                              (rst_n),
    .config_cache_swap                  (config_switch_caches),
    
    // The Pipeline access lines
    .i_if_read_address                  (PC_counter),                // PC address input
    .i_if_pc_update                     (if_pc_update),                   // PC Update and request for new instruction
    .i_config_cache_enabled             (config_cache_enabled),
    
    // The RF stage
    .o_rf_instuction                    (rf_instuction),                  // Output switch for the instruction, the External access or Cache mem goes througha mux for this
    .o_rf_valid                         (rf_valid),                       // This is for HIT's and or external update and next cycle access
    
    // The external interface for Instructions
    .i_external_data_in                 (ram_result),               // this is the input from the data interface
    .o_external_address_out             (external_address_out),           // if the cache misses the address
    .o_external_request                 (external_request),               // this sends out the request to the external interface
    .o_external_cache_burst             (external_cache_burst),           // this tells the external core if there is only 1 instruciton or 4 to come back
    .i_external_instrution_valid        (&{ram_result_valid, ram_result_is_read_instr}),      // this is a valid signal to the FMS
    
    //  data cache/instruction cache interface
    .i_mem_data_cache_address           (mem_data_address[11:2]),
    .i_mem_data_cache_write_enable      (mem_cache_data_mask),
    .i_mem_data_cache_data              (mem_external_fifo_data[31:0]),
    .i_mem_data_cache_q                 (mem_data_cache_input),
    
    // the FIFO core for the MEM core access on both read and writes.
    .i_ram_fifo_rdreq                   (ram_fifo_rdreq),
    .i_ram_fifo_wrreq                   (ram_fifo_wrreq),
    .i_ram_fifo_data                    (mem_external_fifo_data),
    
    .o_ram_fifo_empty                   (ram_fifo_empty),
    .o_ram_fifo_full                    (ram_fifo_full),
    .o_ram_fifo_q                       (ram_fifo_q)
);
    
// external interface

memory_avalon memory_avalon_inst(
    .clk                        (clk),
    .rst_n                      (rst_n),
    
    .ram_fifo_q                 (ram_fifo_q),               //input [66:0]
    .ram_fifo_empty             (ram_fifo_empty),           //input
    .ram_fifo_rdreq             (ram_fifo_rdreq),           //output
    //
    .ram_instr_address          (external_address_out),     //input [31:0]
    .ram_instr_burst            (external_cache_burst),
    .ram_instr_req              (external_request),         //input
    .ram_instr_ack              (ram_instr_ack),            //output
    
    .ram_result_address         (ram_result_address),       //output [31:0]
    .ram_result_valid           (ram_result_valid),         //output
    .ram_result_is_read_instr   (ram_result_is_read_instr), //output
    .ram_result_burstcount      (ram_result_burstcount),    //output [2:0]
    .ram_result                 (ram_result),               //output [31:0]
    
    .mem_little_endian          (mem_little_endian),
    //
    .avm_address                (o_avm_address),              //output [31:0]
    .avm_writedata              (o_avm_writedata),            //output [31:0]
    .avm_byteenable             (o_avm_byteenable),           //output [3:0]
    .avm_burstcount             (o_avm_burstcount),           //output [2:0]
    .avm_write                  (o_avm_write),                //output
    .avm_read                   (o_avm_read),                 //output
    
    .avm_waitrequest            (i_avm_waitrequest),          //input
    .avm_readdatavalid          (i_avm_readdatavalid),        //input
    .avm_readdata               (i_avm_readdata)              //input [31:0]
);

//------------------------------------------------------------------------------

endmodule
