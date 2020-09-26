/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module memory_avalon(
    input               clk,
    input               rst_n,
    
    // [68] Write =1, [67:64] Writemask , [63:32] address , [31:0] Data
    input       [68:0]  ram_fifo_q,
    input               ram_fifo_empty,
    output              ram_fifo_rdreq,
    input               mem_little_endian,
    
    //address and req must be held till ack; on ack address can change
    input       [31:0]  ram_instr_address,
    input               ram_instr_req,
    output reg          ram_instr_ack,
    input               ram_instr_burst,
    
    output reg  [31:0]  ram_result_address,
    output reg          ram_result_valid,
    output reg          ram_result_is_read_instr,
    output reg  [2:0]   ram_result_burstcount,
    output reg  [31:0]  ram_result,
    
    //Avalon master interface
    output reg  [31:0]  avm_address,
    output reg  [31:0]  avm_writedata,
    output reg  [3:0]   avm_byteenable,
    output reg  [2:0]   avm_burstcount,
    output reg          avm_write,
    output reg          avm_read,
    
    input               avm_waitrequest,
    input               avm_readdatavalid,
    input       [31:0]  avm_readdata
);

//------------------------------------------------------------------------------ state machine

localparam [1:0] STATE_IDLE  = 2'd0;
localparam [1:0] STATE_WRITE = 2'd1;
localparam [1:0] STATE_READ  = 2'd2;
reg processing;
wire start_write        = (state == STATE_IDLE || ~(avm_waitrequest)) && ~(ram_fifo_empty || processing) && ram_fifo_q[68] == 1'b1;
wire start_read         = (state == STATE_IDLE || ~(avm_waitrequest)) && ~(ram_fifo_empty || processing) && ram_fifo_q[68] == 1'b0 && readp_possible;
wire start_instr_read   = (state == STATE_IDLE || ~(avm_waitrequest)) && ~(start_write) && ~(start_read) && ram_instr_req && readp_possible &&  ~processing;

assign ram_fifo_rdreq = start_read || start_write;

reg [1:0] state;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       state <= STATE_IDLE;
    else if(start_write)                    state <= STATE_WRITE;
    else if(start_read || start_instr_read) state <= STATE_READ;
    else if(~(avm_waitrequest))             state <= STATE_IDLE;
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n)                                             processing <= 'b0;
    else begin
        if (start_read || start_instr_read)                 processing <= 'b1;
        else if (readp_chain)                               processing <= 'b0; 
    end
end

//------------------------------------------------------------------------------ instruction decoding
    reg [31:0]  instrction_address_correction;
    always @* begin
        casez (ram_instr_address[31:29])
            3'b0zz  : instrction_address_correction <= ram_instr_address;
            3'b100  : instrction_address_correction <= {3'b000, ram_instr_address[28:0]};
            3'b101  : instrction_address_correction <= {3'b000, ram_instr_address[28:0]};
            3'b11z  : instrction_address_correction <= {2'b00,  ram_instr_address[29:0]};
        endcase
    end

//------------------------------------------------------------------------------ Data decoding

    reg [31:0]  data_address_correction;
    always @* begin
        casez (ram_fifo_q[63:61])
            3'b0zz  : data_address_correction <= ram_fifo_q[63:32];
            3'b100  : data_address_correction <= {3'b000, ram_fifo_q[60:32]};
            3'b101  : data_address_correction <= {3'b000, ram_fifo_q[60:32]};
            3'b11z  : data_address_correction <= {2'b00,  ram_fifo_q[61:32]};
        endcase
    end

//------------------------------------------------------------------------------ pipeline read


wire readp_0_update =                                           (start_read || start_instr_read) && readp_2[32:30] == 3'd0 && readp_1[32:30] == 3'd0 && (readp_0[32:30] == 3'd0 || readp_chain);
wire readp_1_update =                      ~(readp_0_update) && (start_read || start_instr_read) && readp_2[32:30] == 3'd0 &&                           (readp_1[32:30] == 3'd0 || readp_chain);
wire readp_2_update = ~(readp_1_update) && ~(readp_0_update) && (start_read || start_instr_read) &&                                                     (readp_2[32:30] == 3'd0 || readp_chain);

wire readp_chain = readp_0[32:30] == 3'd1 && avm_readdatavalid;

wire [2:0]  readp_0_burstcount = readp_0[32:30] - 3'd1;
wire [29:0] readp_0_address    = readp_0[29:0]  + 30'd1;

wire [2:0]  read_burstcount = 3'd1;

wire [33:0] readp_value = (start_read)? { 1'b0, read_burstcount, ram_fifo_q[65:36] } : { 1'b1, {4{ram_instr_burst}}, instrction_address_correction[31:2] };

reg [33:0] readp_0;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                                   readp_0 <= 34'd0;
    else if(readp_0_update)                             readp_0 <= readp_value;
    else if(readp_chain)                                readp_0 <= readp_1;
    else if(readp_0[32:30] > 3'd0 && avm_readdatavalid) readp_0 <= { readp_0[33], readp_0_burstcount, readp_0_address };
end

reg [33:0] readp_1;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)       readp_1 <= 34'd0;
    else if(readp_1_update) readp_1 <= readp_value;
    else if(readp_chain)    readp_1 <= readp_2;
end

reg [33:0] readp_2;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)       readp_2 <= 34'd0;
    else if(readp_2_update) readp_2 <= readp_value;
    else if(readp_chain)    readp_2 <= 34'd0;
end

wire readp_possible = readp_2[32:30] == 3'd0 || readp_1[32:30] == 3'd0 || readp_0[32:30] == 3'd0 || (readp_0[32:30] == 3'd1 && avm_readdatavalid);

//------------------------------------------------------------------------------ avalon bus control

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_address <= 32'd0;
    else if(start_write || start_read)      avm_address <= { data_address_correction[31:2], 2'b00 };
    else if(start_instr_read)               avm_address <= { instrction_address_correction[31:2], 2'b00 }; 
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_writedata <= 32'd0;
    else if(start_write)                    avm_writedata <= mem_little_endian? {ram_fifo_q[7:0],ram_fifo_q[15:8],ram_fifo_q[23:16], ram_fifo_q[31:24]} 
                                                                                : ram_fifo_q[31:0];
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_byteenable <= 4'd0;
    else if(start_write || start_read)      avm_byteenable <= ram_fifo_q[67:64];
    else if(start_instr_read)               avm_byteenable <= 4'hF;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_burstcount <= 3'd0;
    else if(start_write)                    avm_burstcount <= 3'd1;
    else if(start_read)                     avm_burstcount <= read_burstcount;
    else if(start_instr_read)               avm_burstcount <= {4{ram_instr_burst}};
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_read <= 1'b0;
    else if(start_read || start_instr_read) avm_read <= 1'b1;
    else if(~(avm_waitrequest))             avm_read <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0)                       avm_write <= 1'b0;
    else if(start_write)                    avm_write <= 1'b1;
    else if(~(avm_waitrequest))             avm_write <= 1'b0;
end

//------------------------------------------------------------------------------ results

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_instr_ack            <= `FALSE; 
    else ram_instr_ack            <= start_instr_read;         
    end

always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_result_address       <= 32'd0;  
    else ram_result_address       <= { readp_0[29:0], 2'b00 }; 
    end
    
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_result_valid         <= `FALSE; 
    else ram_result_valid         <= avm_readdatavalid;        
    end
    
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_result_burstcount    <= 3'd0;   
    else ram_result_burstcount    <= readp_0[32:30];           
    end
    
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_result_is_read_instr <= 1'b0;   
    else ram_result_is_read_instr <= readp_0[33];              
    end
    
always @(posedge clk or negedge rst_n) begin 
    if(rst_n == 1'b0) ram_result  <= 32'd0;  
    else begin
        if (avm_readdatavalid) ram_result <= mem_little_endian  ? {avm_readdata[7:0], avm_readdata[15:8], avm_readdata[23:16], avm_readdata[31:24]} 
                                                                : avm_readdata;  
        end           
    end


endmodule
