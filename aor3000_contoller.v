`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2020 19:16:47
// Design Name: 
// Module Name: aor3000_contoller
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


module aor3000_contoller(
    input                   clk,
    input                   rst_n,
    
    input                   config_kernel_mode,         // this is for the load and store error checking on the EXE stage
    input                   config_cache_enabled,       // On store to the scrapad this will happen on the EXE stage
    input                   config_isolate_cache,
    input                   config_coproc0_usable,
    input                   config_coproc1_usable,
    input                   config_coproc2_usable,
    
    input       [31:0]      rf_pc_location,
    
    input       [31:0]      rf_instruction,
    input                   rf_valid,
    
    output                  rf_pc_update,
    
    // RF Output
    
    output      [4:0]       rf_address_a,               // Reg Address access for the RS/A side
    output      [4:0]       rf_address_b,               // Reg Address access for the RT/B side
    output                  rf_stall,                   // RF stage stall - Off is the pipeline will move
    input                   i_pc_kill_instuction,
    
    output reg  [1:0]       exe_a_bypass,               // this is the bypass on the exe stage for the RF result to the ALU stage
    output reg  [2:0]       exe_b_bypass,               // this is the bypass on the exe stage or storeage for the RF result to the ALU stage
    
    // EXE stage core
    output                  exe_stall,                  // stall the EXE stage
    output                  exe_clear,                  // clear the EXE Stage
    
    output reg  [26:0]      exe_instant_value,          // Instant for Jumps and ALU calulations

    output reg  [31:0]      exe_pc,                     // This is used for Jumps and the calculations

    output reg  [3:0]       exe_su_alu,                 // selecter for the ALU stage exe
    output reg  [2:0]       exe_su_branch,              // selecter for the branch stage exe
    output reg  [4:0]       exe_su_load,                // selecter for the load stage exe for checking
    output reg  [4:0]       exe_su_store,               // selecter for the storing stage exe for checking
    output reg  [2:0]       exe_su_shift,               // selecter for the shifter in the EXE stage
    output reg  [3:0]       exe_su_result_mux,          // selecter for the result output of the EXE Stage
    output reg              exe_su_shift_in,            // This selects the shifter for instant value to be used for the shift
    output reg              exe_su_signed_alu,          // Selecter for signed numbers

    output reg              exe_mthi,                   // this stores to the HI reg
    output reg              exe_mtlo,                   // this stores to the LO reg
    output reg              exe_mult,                   // this does a multi signed in the EXE stage
    output reg              exe_multu,                  // this does a multi unsigned in the EXE stage
    output reg              exe_div,                    // this does a div signed in the EXE stage
    output reg              exe_divu,                   // this does a div unsigned in the EXE stage
    //
    output reg              exe_load,                   // this does a signal for a load command in the EXE stage
    output reg              exe_store,                  // this does a signal for a store command in the EXE stage  
    output reg              exe_mtc0,      
//    output                  exe_mfc0,                 // this is not needed as we use a MUX to get this out
    output reg              exe_rfe,
        //
    input                   exe_busy,                   // this tells us that the DIV/Multi is still running - 0cycle
    input                   exe_COP2_busy,                   // this tells us that the DIV/Multi is still running - 0cycle
    
    input                   exe_branch_start,
    input                   exe_branch_waiting,
    // MEM Write back

    output reg  [31:0]      mem_pc,                     // This is used for Jumps and the calculations
    
    
    output reg              mem_mtc2,
    output reg              mem_mfc2,
    
    output reg              mem_syscall, 
    output reg              mem_break, 
    output reg              mem_reserved_instr, 
    output reg              mem_coproc_unusable, 

    output                  mem_stall,                  // to stall the MEM stage of the core
    output                  mem_clear,                  // to clear the MEM stage of the core

    output reg              mem_store_enable,           //  this is for the MEM store out to the fifo core
    
    input                   mem_cache_enabled,
   
    output reg  [4:0]       mem_su_load,                // selector for the load commands
    output reg  [4:0]       mem_su_store,               // selector for the Store commands
    
    input                   mem_ram_fifo_write,
    input                   mem_ram_fifo_full,
    
    input                   mem_instrution_result_valid,
    input                   mem_result_valid,
    output reg  [31:0]      mem_instr,
    
    input                   mem_exception_start,        // Here we are to clear this all
 
    //WB Stage   
 
    output reg  [4:0]       wb_result_index             // this is for the writeback to the regs
);

wire              rf_clear;

reg     [31:0]    rf_instruction_holding;

wire  [3:0]       op_su_alu_c;
wire  [2:0]       op_su_branch_c;
wire  [4:0]       op_su_load_c;
wire  [4:0]       op_su_store_c;
wire  [2:0]       op_su_shift_c;
wire  [3:0]       op_su_result_mux_c;
wire  [2:0]       op_su_ls_shift_c;
wire              op_su_shift_in_c;
wire              op_shift_instant_c;
wire              op_signed_alu_c;
wire              rf_load;
wire              rf_exe_multi_div;
wire              rf_exe_cop2_inst;

reg                 exe_ready;
reg                 reset_delay;

assign exe_clear = 0;
assign exe_stall = 0;

// RF to EXE stage Controller

always @(posedge clk) begin
    reset_delay <= rst_n;
end

always @(posedge clk or negedge rst_n or posedge rf_clear) begin
    if (~rst_n || rf_clear) begin
        exe_su_alu          <= 'b0;
        exe_su_branch       <= 'b0;
        exe_su_load         <= 'b0;
        exe_su_store        <= 'b0;
        exe_su_shift        <= 'b0;
        exe_su_result_mux   <= 'b0;
        exe_su_shift_in     <= 'b0;
        exe_su_signed_alu   <= 'b0;
        exe_instant_value   <= 'b0;
        exe_mthi            <= 'b0;
        exe_mtlo            <= 'b0;
        exe_mult            <= 'b0;
        exe_multu           <= 'b0;
        exe_div             <= 'b0;
        exe_divu            <= 'b0;
        exe_load            <= 'b0;
        exe_store           <= 'b0;
        exe_mtc0            <= 'b0;
        exe_rfe             <= 'b0;
        exe_a_bypass        <= 'b0;
        exe_b_bypass        <= 'b0;
        wb_result_index     <= 'b0;
        exe_pc              <= 'b0;
        exe_ready           <= 'b0;
    end
    else if (rf_pc_update && reset_delay && ~i_pc_kill_instuction)begin
        exe_su_alu              <= op_su_alu_c;                 // selecter for the ALU stage exe
        exe_su_branch           <= op_su_branch_c;              // selecter for the branch stage exe
        exe_su_load             <= op_su_load_c;                // selecter for the load stage exe for checking
        exe_su_store            <= op_su_store_c;               // selecter for the storing stage exe for checking
        exe_su_shift            <= op_su_shift_c;               // selecter for the shifter in the EXE stage
        exe_su_result_mux       <= op_su_result_mux_c;          // selecter for the result output of the EXE Stage
        exe_su_shift_in         <= op_su_shift_in_c;            // This selects the shifter for instant value to be used for the shift
        exe_su_signed_alu       <= op_signed_alu_c;             // Selecter for signed numbers
        exe_instant_value       <= rf_instruction[26:0];
        exe_mthi                <= op_mthi_c;
        exe_mtlo                <= op_mtlo_c;
        exe_mult                <= op_mult_c;
        exe_multu               <= op_multu_c;
        exe_div                 <= op_div_c;
        exe_divu                <= op_divu_c;
        exe_load                <= op_load_c;
        exe_store               <= op_store_c;
        exe_mtc0                <= op_mtc0_c;
        exe_rfe                 <= op_rfe_c;
        exe_pc                  <= rf_pc_location;
        exe_ready               <= 'b1;
    end
    else if (~rf_stall) begin
        exe_su_alu              <= 'b0;                 // selecter for the ALU stage exe
        exe_su_branch           <= 'b0;              // selecter for the branch stage exe
        exe_su_load             <= 'b0;                // selecter for the load stage exe for checking
        exe_su_store            <= 'b0;               // selecter for the storing stage exe for checking
        exe_su_shift            <= 'b0;               // selecter for the shifter in the EXE stage
        exe_su_result_mux       <= 'b0;          // selecter for the result output of the EXE Stage
        exe_su_shift_in         <= 'b0;            // This selects the shifter for instant value to be used for the shift
        exe_su_signed_alu       <= 'b0;             // Selecter for signed numbers
        exe_instant_value       <= 'b0;
        exe_mthi                <= 'b0;
        exe_mtlo                <= 'b0;
        exe_mult                <= 'b0;
        exe_multu               <= 'b0;
        exe_div                 <= 'b0;
        exe_divu                <= 'b0;
        exe_load                <= 'b0;
        exe_store               <= 'b0;
        exe_mtc0                <= 'b0;
        exe_rfe                 <= 'b0;
        exe_ready               <= 'b0;
    end
end



always @(posedge clk or negedge rst_n or posedge rf_clear) begin
    if (~rst_n || rf_clear) begin
        rf_instruction_holding <= 'b0;
    end
    else begin
        if (rf_stall && rf_valid) rf_instruction_holding <= rf_instruction;
    end
end

assign rf_address_a = (rf_recover?  rf_instruction_holding[25:20] : rf_instruction[25:20]);
assign rf_address_b = (rf_recover?  rf_instruction_holding[19:15] : rf_instruction[19:15]);

always @(posedge clk or negedge rst_n) begin
    
end

rf_fms_core rf_fms_core (
    .clk                    (clk),
    .rst_n                  (rst_n),
    
    .rf_clear               (rf_clear),
    .rf_valid               (rf_valid),
    .rf_load                (rf_load),
    .rf_pc_update           (rf_pc_update),
    .rf_stall               (rf_stall),
    .rf_recover             (rf_recover),
    
    .rf_exe_multi_div       (rf_exe_multi_div),
    .rf_exe_cop2_inst       (rf_exe_cop2_inst),
    
    .exe_busy               (exe_busy),
    .exe_COP2_busy          (exe_COP2_busy),
    
    .mem_stall              (mem_stall),
    
    .mem_exception_start    (mem_exception_start)
);

rf_rom_core rf_rom_core (
        .if_RF              (rf_recover ? rf_instruction_holding : rf_instruction),
        .op_su_alu_c        (op_su_alu_c),
        .op_su_branch_c     (op_su_branch_c),
        .op_su_load_c       (op_su_load_c),
        .op_su_store_c      (op_su_store_c),
        .op_su_shift_c      (op_su_shift_c),
        .op_su_result_mux_c (op_su_result_mux_c),
        .op_su_ls_shift_c   (op_su_ls_shift_c),
        .op_su_shift_in_c   (op_su_shift_in_c),
        .op_shift_instant_c (op_shift_instant_c),
        .op_signed_alu_c    (op_signed_alu_c),
        .op_rf_load_c       (rf_load),
        .op_rf_COP2_c       (rf_exe_cop2_inst),
        .op_rf_exe_multi_c  (rf_exe_multi_div),
        .op_mthi_c          (op_mthi_c),                   // this stores to the HI reg
        .op_mtlo_c          (op_mtlo_c),                   // this stores to the LO reg
        .op_mult_c          (op_mult_c),                   // this does a multi signed in the EXE stage
        .op_multu_c         (op_multu_c),                  // this does a multi unsigned in the EXE stage
        .op_div_c           (op_div_c),                    // this does a div signed in the EXE stage
        .op_divu_c          (op_divu_c),                   // this does a div unsigned in the EXE stage
        .op_load_c          (op_load_c),                   // this does a signal for a load command in the EXE stage
        .op_store_c         (op_store_c),                  // this does a signal for a store command in the EXE stage  
        .op_mtc0_c          (op_mtc0_c),
        .op_rfe_c           (op_rfe_c)
);





exe_mem_fms_core exe_mem_fms_core (

);

exe_mem_rom_core exe_mem_rom_core (

);

endmodule


module rf_rom_core (
        input       [31:0]      if_RF,
        output reg  [3:0]       op_su_alu_c,
        output reg  [2:0]       op_su_branch_c,
        output reg  [4:0]       op_su_load_c,
        output reg  [4:0]       op_su_store_c,
        output reg  [2:0]       op_su_shift_c,
        output reg  [3:0]       op_su_result_mux_c,
        output reg  [2:0]       op_su_ls_shift_c,
        output reg              op_su_shift_in_c,
        output reg              op_shift_instant_c,
        output reg              op_signed_alu_c,
        output reg              op_rf_load_c,
        output reg              op_rf_COP2_c,
        output reg              op_rf_exe_multi_c,
        output reg              op_mthi_c,
        output reg              op_mtlo_c,
        output reg              op_mult_c,
        output reg              op_multu_c,
        output reg              op_div_c,
        output reg              op_divu_c,
        output reg              op_load_c,
        output reg              op_store_c,  
        output reg              op_mtc0_c,      
//    output                  exe_mfc0,                 // this is not needed as we use a MUX to get this out
        output reg              op_rfe_c
);

`include "defines.v"

// rf test for Load commands

always @* begin
    case (if_RF[31:29] )
        3'b010  ,
        3'b100  ,
        3'b110  : op_rf_load_c <= 'b1;
        default : op_rf_load_c <= 'b0;
    endcase
end

always @* begin
    case (if_RF[31:21] )
        11'b010_000_00000 : op_mtc0_c <= 'b1;
        default : op_mtc0_c <= 'b0;
    endcase
end

// cop2

always @* begin
    case (if_RF[31:26] )
        6'h12   : op_rf_COP2_c <= 'b1;
        default : op_rf_COP2_c <= 'b0;
    endcase
end

// EXE Multi

always @* begin
    case ({if_RF[31:26], if_RF[5:4]})
        8'b000_000_01   : op_rf_exe_multi_c <= 'b1;
        default         : op_rf_exe_multi_c <= 'b0;
    endcase
end

// EXE mthi

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_010_001  : op_mthi_c <= 'b1;
        default             : op_mthi_c <= 'b0;
    endcase
end

// EXE mtlo

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_010_011  : op_mtlo_c <= 'b1;
        default             : op_mtlo_c <= 'b0;
    endcase
end

// EXE op_mult_c

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_011_000  : op_mult_c <= 'b1;
        default             : op_mult_c <= 'b0;
    endcase
end

// EXE op_multu_c

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_011_001  : op_multu_c <= 'b1;
        default             : op_multu_c <= 'b0;
    endcase
end

// EXE op_div_c

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_011_010  : op_div_c <= 'b1;
        default             : op_div_c <= 'b0;
    endcase
end

// EXE op_divu_c

always @* begin
    case ({if_RF[31:26], if_RF[5:0]})
        12'b000_000_011_011  : op_divu_c <= 'b1;
        default             : op_divu_c <= 'b0;
    endcase
end

// EXE op_rfe_c

always @* begin
    case ({if_RF[31:25], if_RF[5:0]})
        13'b010_000_1_010_000  : op_rfe_c <= 'b1;
        default             : op_rfe_c <= 'b0;
    endcase
end

//ALU Controller



always @* begin
	casez ({if_RF[31:26], if_RF[5:0]})
	    12'b001_111_zzz_zzz	: op_su_alu_c <= `APU_LUI; // LUI
		12'b001_00z_zzz_zzz	: op_su_alu_c <= `APU_ADD; // ADDI and ADDIU
		12'b001_01z_zzz_zzz	: op_su_alu_c <= `APU_SLT; // SLTI and SLTIU
		12'b001_100_zzz_zzz	: op_su_alu_c <= `APU_AND; // ANDI
		12'b001_101_zzz_zzz	: op_su_alu_c <= `APU_OR ; // ORI
		12'b001_110_zzz_zzz	: op_su_alu_c <= `APU_XOR; // XORI
		// Special Area
		12'b000_000_100_00z	: op_su_alu_c <= `APU_ADD; // ADD and ADDU
		12'b000_000_100_01z	: op_su_alu_c <= `APU_SUB; // SLT and SLTU
		12'b000_000_100_100	: op_su_alu_c <= `APU_AND; // AND
		12'b000_000_100_101	: op_su_alu_c <= `APU_OR ; // OR
		12'b000_000_100_110	: op_su_alu_c <= `APU_XOR; // XOR
		12'b000_000_100_111	: op_su_alu_c <= `APU_NOR; // NOR
		12'b000_000_101_01z	: op_su_alu_c <= `APU_SLT;
		default				: op_su_alu_c <= `NONE;
	endcase
end

//Branch Controller
always @* begin
	casez ({if_RF[31:26], if_RF[20:16], if_RF[5:0]})
		17'b000_01z_zzz_zz_zzz_zzz	: op_su_branch_c <= `BRANCH_JUMP; // Jump and Jump link
		17'b000_100_zzz_zz_zzz_zzz	: op_su_branch_c <= `BRANCH_EQ; // Branch Equal
		17'b000_101_zzz_zz_zzz_zzz	: op_su_branch_c <= `BRANCH_NEQ; // Branch not Equal
		17'b000_110_zzz_zz_zzz_zzz	: op_su_branch_c <= `BRANCH_LEZ; // Branch Less and = to zero
		17'b000_111_zzz_zz_zzz_zzz	: op_su_branch_c <= `BRANCH_GZ; // Branch Greater to zero
		17'b000_000_zzz_zz_001_00z	: op_su_branch_c <= `BRANCH_JUMP_R; // JUMP Regature and jump reg and link
		17'b000_001_00_000_zzz_zzz	: op_su_branch_c <= `BRANCH_LZ; // less than zero
		17'b000_001_00_001_zzz_zzz	: op_su_branch_c <= `BRANCH_GZ; // greater than zero
		17'b000_001_10_000_zzz_zzz	: op_su_branch_c <= `BRANCH_GZ; // greater than zero
		17'b000_001_10_001_zzz_zzz	: op_su_branch_c <= `BRANCH_LZ; // less than zero
		default                     : op_su_branch_c <= `NONE; // none
		// Special Area
	endcase
end

//Load Controller
always @* begin
	casez ({if_RF[31:26], if_RF[25:21], if_RF[15:11]})
		16'b100_100_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_UBYTE; // Unsign Byte
		16'b100_000_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_SBYTE; // Sign Byte
		16'b100_101_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_UHALF; // Unsign Half
		16'b100_001_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_SHALF; // Sign Half
		16'b100_011_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_WORD;  // Word
		16'b100_110_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_LWR;  // Word
		16'b100_010_zz_zzz_zz_zzz	: op_su_load_c <= `LOAD_LWL;  // Word
        16'b010_000_00_000_zz_zzz	: op_su_load_c <= `LOAD_WORD; // MFC0 Load
		16'b010_010_00_000_zz_zzz	: op_su_load_c <= `LOAD_WORD; // MFC2 Load
		default                     : op_su_load_c <= `NONE;
	endcase
end

always @* begin
	casez ({if_RF[31:26], if_RF[25:21], if_RF[15:11]})
		16'b100_zzz_zz_zzz_zz_zzz	: op_load_c <= 'b1; // Load line
		default                     : op_load_c <= 'b0;
	endcase
end

//Store Controller
always @* begin
	casez ({if_RF[31:26], if_RF[25:21], if_RF[15:11]})
		16'b101_000_zz_zzz_zz_zzz	: op_su_store_c <= `STORE_BYTE; // Sign Byte
		16'b101_001_zz_zzz_zz_zzz	: op_su_store_c <= `STORE_HALF; // Sign Half
		16'b101_011_zz_zzz_zz_zzz	: op_su_store_c <= `STORE_WORD;  // Word
		default                     : op_su_store_c <= `NONE;
	endcase
end

always @* begin
	casez ({if_RF[31:26], if_RF[25:21], if_RF[15:11]})
		16'b101_zzz_zz_zzz_zz_zzz	: op_store_c <= 'b1; // Load line
		default                     : op_store_c <= 'b0;
	endcase
end

//Result Mux Controller
always @* begin
	casez ({if_RF[31:16], if_RF[5:0]})
		22'b000_011_zz_zzz_zzz_zz_zzz_zzz	: op_su_result_mux_c <= `SU_RESULT_PC; // PC Result for link
		22'b001_zzz_zz_zzz_zzz_zz_zzz_zzz	: op_su_result_mux_c <= `SU_RESULT_ALU; // ALU Result
		22'b100_zzz_zz_zzz_zzz_zz_zzz_zzz	: op_su_result_mux_c <= `SU_RESULT_LOAD; // Load result
		22'b000_000_zz_zzz_zzz_zz_000_zzz	: op_su_result_mux_c <= `SU_RESULT_SHIFT; // Shift result
		22'b000_000_zz_zzz_zzz_zz_001_001	: op_su_result_mux_c <= `SU_RESULT_PC; // PC Result for link
		22'b000_000_zz_zzz_zz_zzz_100_zzz	: op_su_result_mux_c <= `SU_RESULT_ALU; // ALU Result
		22'b000_001_zz_zzz_10_00z_zzz_zzz	: op_su_result_mux_c <= `SU_RESULT_PC; // PC Result for link
		default                             : op_su_result_mux_c <= `NONE; // XORI
		// Special Area
	endcase
end

//Result Mux Controller
always @* begin
	casez (if_RF[5:0])
		6'b000_000	: begin
		                  op_su_shift_c <= `SU_SHIFT_SLL; // Shift Left using Logical with zeros Instant
		                  op_su_shift_in_c <= 1'b1;
                      end
		6'b000_010	: begin    
		                  op_su_shift_c <= `SU_SHIFT_SRL; // Shift Right using Logical with zeros Instant
		                  op_su_shift_in_c <= 1'b1;
                      end
		6'b000_011	: begin    
		                  op_su_shift_c <= `SU_SHIFT_SRA; // Shift Right using Arithmetic signed Instant
		                  op_su_shift_in_c <= 1'b1;
                      end
		6'b000_100	: begin    
		                  op_su_shift_c <= `SU_SHIFT_SLLV; // Shift Left using Logical with zeros and RF_A
		                  op_su_shift_in_c <= 1'b0;
                      end
		6'b000_110	: begin    
		                  op_su_shift_c <= `SU_SHIFT_SRLV; // Shift Right sing Logical with zeros and RF_A
		                  op_su_shift_in_c <= 1'b0;
                      end
		6'b000_111	: begin    
		                  op_su_shift_c <= `SU_SHIFT_SRAV; // Shift Right using Arithmetic signed and RF_A
		                  op_su_shift_in_c <= 1'b0;
                      end
		default     : begin    
		                  op_su_shift_c <= `NONE; // Branch Greater to zero
		                  op_su_shift_in_c <= 1'b0;
                      end
		// Special Area
	endcase
end

// Instant contorlller

always @* begin
	casez ({if_RF[31:26], if_RF[5:0]})
		6'b000_000_000_0zz	: op_shift_instant_c <= 1'b1; // Instant rf_instr[15:0]
		default             : op_shift_instant_c <= `NONE; // RD result
		// Special Area
	endcase
end

always @* begin
	casez ({if_RF[31:26], if_RF[5:0]})
		12'b001_000_zzz_zzz	: op_signed_alu_c <= 1'b1; // ADDI
		12'b001_010_zzz_zzz	: op_signed_alu_c <= 1'b1; // SLTI
		12'b000_000_100_000	: op_signed_alu_c <= 1'b1; // ADD
		12'b000_000_100_010	: op_signed_alu_c <= 1'b1; // SUB
		default             : op_signed_alu_c <= 1'b0; // XORI 
    endcase
end

endmodule

module exe_mem_fms_core (

);

endmodule

module exe_mem_rom_core (

);

endmodule

module hazord_core (
    input   [31:0]  if_RF
);

reg     instant_rt_c;

//Result Mux Controller
always @* begin
	casez (if_RF[31:26])
		6'b001_zzz	: instant_rt_c <= 1'b1; // Instant rf_instr[15:0]
		default     : instant_rt_c <= `NONE; // XORI
		// Special Area
	endcase
end

endmodule