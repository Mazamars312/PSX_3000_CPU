/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2020   Murray Aickin
 */

//------------------------------------------------------------------------------

`define TRUE        1'b1
`define FALSE       1'b0
`define NONE        'b0

//------------------------------------------------------------------------------

/*******************************************************************************

    RF Stage defines

********************************************************************************/

`define rf_rb_bypass        3'b001  // Select the Final write for the bypass
`define rf_mem_bypass       3'b010  // Select the MEM Stage for the bypass
`define rf_exe_bypass       3'b011  // Select the EXE Stage for the bypass
`define rf_inst_bypass      3'b100  // Select the Instant for the bypass


//APU

`define APU_ADD         4'd01
`define APU_SLT         4'd02
`define APU_AND         4'd03
`define APU_OR          4'd04
`define APU_XOR         4'd05
`define APU_SUB         4'd06
`define APU_NOR         4'd07
`define APU_LUI         4'd08
`define APU_HI_R        4'd09
`define APU_HI_W        4'd10
`define APU_LO_R        4'd11
`define APU_LO_W        4'd12

//BRANCH

`define BRANCH_JUMP     3'd01
`define BRANCH_JUMP_R   3'd02
`define BRANCH_EQ       3'd03
`define BRANCH_NEQ      3'd04
`define BRANCH_LEZ      3'd05
`define BRANCH_GZ       3'd06
`define BRANCH_LZ       3'd07

// LOAD Cores

`define LOAD_UBYTE      4'd01
`define LOAD_SBYTE      4'd02
`define LOAD_UHALF      4'd03
`define LOAD_SHALF      4'd04
`define LOAD_WORD       4'd05
`define LOAD_LWR        4'd06
`define LOAD_LWL        4'd07
`define LOAD_COP0       4'd08
`define LOAD_COP2       4'd09

// STORE Cores

`define STORE_BYTE      3'd01
`define STORE_HALF      3'd02
`define STORE_WORD      3'd03
`define STORE_SWR       3'd04
`define STORE_SWL       3'd05
`define STORE_COP0      3'd06
`define STORE_COP2      3'd06

// EXE Result

`define SU_RESULT_A     4'd1
`define SU_RESULT_INST  4'd2
`define SU_RESULT_ALU   4'd3
`define SU_RESULT_SHIFT 4'd4
`define SU_RESULT_PC    4'd5
`define SU_RESULT_LOAD  4'd6
`define SU_RESULT_CP0   4'd7
`define SU_RESULT_CP2M  4'd8
`define SU_RESULT_CP2C  4'd9
`define SU_RESULT_HI    4'd10
`define SU_RESULT_LO    4'd11

// Shift controller

`define SU_SHIFT_SLL    3'd1
`define SU_SHIFT_SRL    3'd2
`define SU_SHIFT_SRA    3'd3
`define SU_SHIFT_SLLV   3'd4
`define SU_SHIFT_SRLV   3'd5
`define SU_SHIFT_SRAV   3'd6
