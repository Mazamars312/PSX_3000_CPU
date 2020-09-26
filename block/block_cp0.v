/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module aor3000_block_cp0(
    input               clk,
    input               rst_n,
    //
    input       [5:0]   interrupt_vector,
    
    //
    output reg          config_switch_caches,
    output reg          config_isolate_cache,
    output              config_coproc0_usable,
    output              config_coproc1_usable,
    output              config_coproc2_usable,
    output              config_kernel_mode,
    output reg          mem_little_endian,
    //
    input   [26:0]      exe_instr,
    input   [31:0]      exe_b,
    input               exe_mtc0,
    input               exe_rfe,
    //
    input               mem_store_address_error,
    input               mem_load_address_error,
    input               mem_syscall, 
    input               mem_break, 
    input               mem_reserved_instr, 
    input               mem_coproc_unusable, 
    input               mem_int_overflow,
    
    output reg  [31:0]  mem_coproc0_output,
 
    //
    input               sr_cm_set,
    input               sr_cm_clear,

    //
    output              exception_start,
    output reg  [31:0]  exception_start_pc,
    
    //
    input               mem_stall,
    input       [31:0]  mem_instr,
    input       [31:0]  mem_pc,
    input       [1:0]   mem_branched,
    input       [31:0]  mem_branch_address,
    input       [31:0]  mem_address
    
); 

//------------------------------------------------------------------------------

reg [5:0]  sr_ku_ie;            //kernel/user and interrupt enable
reg        sr_bev;              //boot exception vector
reg        sr_cm;               //last d-cache load hit; used in d-cache isolated
reg [7:0]  sr_im;               //interrupt mask
reg [3:0]  sr_coproc_usable;    //coprocessor usable

reg        sr_tlb_shutdown;
reg        sr_parity_error;
reg        sr_parity_zero;

reg [31:0] BPC;
reg [31:0] BDA;
reg [31:0] BPCM;
reg [31:0] BDAM;
reg [31:0] JUMPDEST;

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) BPC      <= 4'b0;   
    else if(exe_mtc0 && exe_instr[15:11] == 5'd3) BPC     <= exe_b; 
    end
    
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) BDA      <= 4'b0;   
    else if(exe_mtc0 && exe_instr[15:11] == 5'd5) BDA     <= exe_b; 
    end

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) BPCM      <= 4'b0;   
    else if(exe_mtc0 && exe_instr[15:11] == 5'd11) BPCM     <= exe_b; 
    end

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) BDAM      <= 4'b0;   
    else if(exe_mtc0 && exe_instr[15:11] == 5'd9) BDAM     <= exe_b; 
    end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   sr_ku_ie <= 6'b0;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12)      sr_ku_ie <= exe_b[5:0];
    else if(exe_rfe)                                    sr_ku_ie <= { sr_ku_ie[5:4], sr_ku_ie[5:2] };
    else if(exception_start)                            sr_ku_ie <= { sr_ku_ie[3:0], 2'b00 };
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   sr_cm <= `FALSE;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12)      sr_cm <= exe_b[19];
    else if(sr_cm_clear)                                sr_cm <= `FALSE; //first sr_cm_clear important
    else if(sr_cm_set)                                  sr_cm <= `TRUE;
end

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_coproc_usable      <= 4'b0;   
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_coproc_usable     <= exe_b[31:28]; 
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) mem_little_endian     <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) mem_little_endian    <= exe_b[25];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_bev                <= `TRUE;  
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_bev               <= exe_b[22];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_tlb_shutdown       <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_tlb_shutdown      <= exe_b[21];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_parity_error       <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_parity_error      <= exe_b[20];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_parity_zero        <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_parity_zero       <= exe_b[18];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) config_switch_caches  <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) config_switch_caches <= exe_b[17];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) config_isolate_cache  <= `FALSE; 
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) config_isolate_cache <= exe_b[16];    
    end
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) sr_im                 <= 8'h00;  
    else if(exe_mtc0 && exe_instr[15:11] == 5'd12) sr_im                <= exe_b[15:8];  
    end

assign config_kernel_mode    = ~(sr_ku_ie[1]);
assign config_coproc0_usable = sr_coproc_usable[0];
assign config_coproc1_usable = sr_coproc_usable[1];
assign config_coproc2_usable = sr_coproc_usable[2];

//------------------------------------------------------------------------------

reg        cause_bd;            //branch delay
reg [1:0]  cause_ce;            //coproc error
reg [1:0]  cause_ip_writable;   //interrupt pending ([1:0] writable)
reg [4:0]  cause_exccode;       //exccode
reg [31:0] epc;
reg [31:0] badvaddr;

reg [5:0] interrupt_vector_reg;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)   interrupt_vector_reg <= 6'd0;
    else                interrupt_vector_reg <= interrupt_vector;
end

wire [7:0] cause_ip = { interrupt_vector_reg, cause_ip_writable };

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   cause_bd <= `FALSE;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd13)      cause_bd <= exe_b[31];
    else if(exception_start)                            cause_bd <= (exception_not_interrupt && mem_branched == 2'd2) || (exception_interrupt && mem_branched == 2'd1);
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   cause_ce <= 2'd0;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd13)      cause_ce <= exe_b[29:28];
    else if(mem_coproc_unusable)                        cause_ce <= mem_instr[27:26];
    else if(exception_start)                            cause_ce <= 2'd0;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   cause_ip_writable <= 2'd0;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd13)      cause_ip_writable <= exe_b[9:8];
    else if(exception_start)                            cause_ip_writable <= 2'd0;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   cause_exccode <= 5'd31;
    else if(exe_mtc0 && exe_instr[15:11] == 5'd13)      cause_exccode <= exe_b[6:2];
    else if(exception_start)                            cause_exccode <= exception_cause;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   epc <= 32'd0;
    else if(exception_start)                            epc <= exception_epc;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                           badvaddr <= 32'd0;
    else if(|{mem_load_address_error, mem_store_address_error}) badvaddr <= mem_address;
end



//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) mem_coproc0_output <= 'b0;
    else begin
        case (exe_instr[15:11])
            5'd3    : mem_coproc0_output <= BPC;
            5'd5    : mem_coproc0_output <= BDA ;
            5'd6    : mem_coproc0_output <= JUMPDEST ;
//            5'd7    : mem_coproc0_output <= DCIC ;
            5'd8    : mem_coproc0_output <= badvaddr; // Breakpoint on Control
            5'd9    : mem_coproc0_output <= BDAM ;
            5'd11   : mem_coproc0_output <= BPCM ;
            5'd12   : mem_coproc0_output <= { sr_coproc_usable, 2'b0, mem_little_endian, 2'b0, sr_bev, sr_tlb_shutdown, sr_parity_error, sr_cm, sr_parity_zero,
                                          config_switch_caches, config_isolate_cache, sr_im, 2'b0, sr_ku_ie };     //SR
            5'd13   : mem_coproc0_output <= { cause_bd, 1'b0, cause_ce, 12'd0, cause_ip, 1'b0, cause_exccode, 2'b0 };
            5'd14   : mem_coproc0_output <= epc;
            5'd15   : mem_coproc0_output <= { 16'd0, 8'h02, 8'h30 };
            default : mem_coproc0_output <= 'b0;
        endcase
    end
end


//------------------------------------------------------------------------------

//input       [6:0]   mem_cmd,
//input               mem_branched,
//input       [31:0]  mem_badvpn

reg mem_stalled_last;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)   mem_stalled_last <= `FALSE;
    else                mem_stalled_last <= mem_stall;
end

wire exception_interrupt = sr_ku_ie[0] && (cause_ip & sr_im) != 8'd0 && ((mem_stalled_last && ~(mem_stall))) && ~(exception_not_interrupt) && ~(mem_stall);

wire exception_not_interrupt = |{mem_load_address_error, mem_store_address_error, mem_syscall, mem_break, mem_reserved_instr, mem_coproc_unusable, mem_int_overflow};

assign exception_start = 0;//exception_not_interrupt || exception_interrupt;

wire [31:0] exception_epc =
    (exception_interrupt && mem_branched == 2'd2)?  mem_branch_address :
    (exception_interrupt && mem_branched == 2'd0)?  mem_pc :
    (mem_branched == 2'd2)?                         mem_pc - 32'd8:
                                                    mem_pc - 32'd4;

always @* begin
    casez ({exception_interrupt, mem_branched})
        3'b1zzz_zzz    :   exception_cause <= 5'd4;
        3'b01zz_zzz    :   exception_cause <= 5'd5;
        3'b001z_zzz    :   exception_cause <= 5'd8;
        3'b0001_zzz    :   exception_cause <= 5'd9;
        
        default        :   exception_cause <= 5'd0;
    endcase
end

reg [4:0]   exception_cause;
                                                                
always @* begin
    casez ({mem_load_address_error, mem_store_address_error, mem_syscall, mem_break, mem_reserved_instr, mem_coproc_unusable, mem_int_overflow})
        7'b1zzz_zzz    :   exception_cause <= 5'd4;
        7'b01zz_zzz    :   exception_cause <= 5'd5;
        7'b001z_zzz    :   exception_cause <= 5'd8;
        7'b0001_zzz    :   exception_cause <= 5'd9;
        7'b0000_1zz    :   exception_cause <= 5'd10;
        7'b0000_01z    :   exception_cause <= 5'd11;
        7'b0000_001    :   exception_cause <= 5'd12;
        default        :   exception_cause <= 5'd0;
    endcase
end
                                                                
always @(*) begin
    case ({sr_bev, mem_load_address_error, mem_store_address_error})
        3'b111  : exception_start_pc <= 32'hBFC00100 ;
        3'b101  : exception_start_pc <= 32'hBFC00100 ;
        3'b110  : exception_start_pc <= 32'hBFC00100 ;
        3'b100  : exception_start_pc <= 32'hBFC00180 ;
        3'b011  : exception_start_pc <= 32'h80000000 ;
        3'b001  : exception_start_pc <= 32'h80000000 ;
        3'b010  : exception_start_pc <= 32'h80000000 ;
        default : exception_start_pc <= 32'h80000080 ;
    endcase
end

endmodule
