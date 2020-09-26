/*
 * This file is subject to the terms and conditions of the BSD License. See
 * the file "LICENSE" in the main directory of this archive for more details.
 *
 * Copyright (C) 2014 Aleksander Osman
 */

`include "defines.v"

module memory_ram(
    input               clk,
    input               rst_n,
    input               config_cache_swap,
    
    // The Pipeline access lines
    input       [31:0]  i_if_read_address,  // PC address input
    input               i_if_pc_update,     // PC Update and request for new instruction
    input               i_config_cache_enabled,
    
    // The RF stage
    output reg  [31:0]  o_rf_instuction,    // Output switch for the instruction, the External access or Cache mem goes througha mux for this
    output reg          o_rf_valid,         // This is for HIT's and or external update and next cycle access
    
    // The external interface for Instructions
    input       [31:0]  i_external_data_in,             // this is the input from the data interface
    output reg  [31:0]  o_external_address_out,         // if the cache misses the address
    output reg          o_external_request,             // this sends out the request to the external interface
    output reg          o_external_cache_burst,         // this tells the external core if there is only 1 instruciton or 4 to come back
    input               i_external_instrution_valid,    // this is a valid signal to the FMS
    
    //  data cache/instruction cache interface
    input       [9:0]   i_mem_data_cache_address,
    input       [3:0]   i_mem_data_cache_write_enable,
    input       [31:0]  i_mem_data_cache_data,
    output reg  [31:0]  i_mem_data_cache_q,
    
    // the FIFO core for the MEM core access on both read and writes.
    input               i_ram_fifo_rdreq,
    input               i_ram_fifo_wrreq,
    input       [68:0]  i_ram_fifo_data,
    
    output              o_ram_fifo_empty,
    output              o_ram_fifo_full,
    output      [68:0]  o_ram_fifo_q
);

//------------------------------------------------------------------------------

wire            cache_hit;
wire [20:0]     instruction_cache_tag;
wire [31:0]     o_instrution_data;
wire [31:0]     o_data_data;
reg  [31:0]     pc_delay;
reg  [1:0]      cache_status, cache_status_c;
reg  [31:0]     o_external_address_out_c;
reg             rf_cache_enable_output;
reg             if_pc_update;
reg             instr_write_external;

parameter   cache_idle      = 2'd0;
parameter   cache_external  = 2'd1;
parameter   cache_refull    = 2'd2;




always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        cache_status <= cache_idle;
        pc_delay <= 'b0;
        o_external_address_out <= 'b0;
        if_pc_update <= 'b0;
    end
    else begin
        cache_status <= cache_status_c;
        pc_delay    <= i_if_read_address;
        o_external_address_out <= ((i_config_cache_enabled && ~config_cache_swap) ? {o_external_address_out_c[31:4], 4'd0} : o_external_address_out_c);
        if_pc_update <= i_if_pc_update;
    end
end

// status chager for the cache system

assign cache_hit = &{(pc_delay[31:12] == instruction_cache_tag[19:0]), instruction_cache_tag[20], i_config_cache_enabled};

always @* begin
    case (cache_status)
        cache_idle      : begin
                        o_external_address_out_c <= i_if_read_address;
                        o_external_request <='b0;
                        o_rf_valid <= 'b0;
                        o_external_cache_burst <= 'b0;
                        rf_cache_enable_output <= 'b0;
                        instr_write_external <= 'b0;
                        if (i_config_cache_enabled && ~config_cache_swap) begin
                            if (if_pc_update && cache_hit) begin
                                cache_status_c <= cache_idle;
                                o_rf_valid <= 'b1;
                                rf_cache_enable_output <= 'b1;
                            end
                            else if (if_pc_update) begin
                                cache_status_c <= cache_external;
                            end
                        end
                        else if (if_pc_update) begin
                            cache_status_c <= cache_external;
                        end
                        else cache_status_c <= cache_idle;
                    end
        cache_external  : begin
                        o_external_address_out_c <= o_external_address_out;
                        o_rf_valid <= 'b0;
                        rf_cache_enable_output <= 'b0;
                        o_external_cache_burst <= 'b0;
                        cache_status_c <= cache_external;
                        instr_write_external <= 'b0;
                        if (i_config_cache_enabled && ~config_cache_swap) begin
                            if (i_external_instrution_valid) begin
                                cache_status_c <= cache_refull;
                                o_external_request <='b0;
                                o_external_address_out_c <= o_external_address_out + 4;
                            end
                            else begin
                                
                                o_external_request <='b1;
                                o_external_cache_burst <= 'b1;
                            end
                        end
                        else begin
                            if (i_external_instrution_valid) begin
                                cache_status_c <= cache_idle;
                                o_external_request <='b0;
                                o_rf_valid <= 'b1;
                            end
                            else begin
                                
                                o_external_request <='b1;
                            end
                        end
                    end
        cache_refull    : begin
                        
                        o_external_request <='b0;
                        o_rf_valid <= 'b0;
                        o_external_address_out_c <= o_external_address_out;
                        rf_cache_enable_output <= 'b0;
                        o_external_cache_burst <= 'b0;
                        instr_write_external <= 'b0;
                        if (i_external_instrution_valid && (o_external_address_out[4:2] == 2'b11)) begin
                            cache_status_c <= cache_idle;
                            instr_write_external <= 'b1;
                        end
                        else if (i_external_instrution_valid) begin
                            cache_status_c <= cache_refull;
                            instr_write_external <= 'b1;
                            o_external_address_out_c <= o_external_address_out + 4;
                        end
                        else begin
                            cache_status_c <= cache_refull;
                        end
                    end
        default         : begin
                        o_external_address_out_c <= o_external_address_out;
                        cache_status_c <= cache_idle;
                        o_external_request <='b0;
                        o_rf_valid <= 'b0;
                        rf_cache_enable_output <= 'b0;
                        o_external_cache_burst <= 'b0;
                        instr_write_external <= 'b0;
                    end
    endcase
end

// instruction Cache

wire [20:0] cache_inst_tag_data = config_cache_swap ? 21'd0 : {1'b1, i_if_read_address};
wire [31:0] cache_inst_data     = config_cache_swap ? i_mem_data_cache_data : i_external_data_in;
wire [9:0]  cache_inst_address  = config_cache_swap ? i_mem_data_cache_address : o_external_address_out;

cache_true_dual_ram #(
    .width          (21),
    .widthad        (8)
)
cache_1_inst_tag(
    .clk            (clk),
    
    //
    .address_a      (i_if_read_address[11:4]),  //input [9:0]
    .q_a            (instruction_cache_tag),   //output [53:0]
    
    //
    .address_b      (cache_inst_address),  //input [9:0]
    .wren_b         (|{(config_cache_swap && |{i_mem_data_cache_write_enable}), instr_write_external}),    //input
    .data_b         (cache_inst_tag_data) //input [53:0]
);

cache_true_dual_ram #(
    .width          (32),
    .widthad        (10)
)
cache_1_inst_data(
    .clk            (clk),
    .address_a      (i_if_read_address[11:2]),
    .q_a            (o_instrution_data),   
    //
    .address_b      (cache_inst_address),
    .wren_b         (|{(config_cache_swap && |{i_mem_data_cache_write_enable}), instr_write_external}),
    .data_b         (cache_inst_data)
);

// This is the data cache and we dont want the IF stage to thing there is anything here

model_data_cache_memory model_data_cache_memory(
    .clk            (clk),
    //
    .address_a      (i_mem_data_cache_address),
    .wren_a         ({{4{~config_cache_swap}} & i_mem_data_cache_write_enable}),
    .q_a            (o_data_data),
    .data_a         (i_mem_data_cache_data)
);

wire [31:0]     data_data, instrution_data;

always @* begin
    case (rf_cache_enable_output)
        1'b1    : o_rf_instuction <= o_instrution_data;
        default : o_rf_instuction <= i_external_data_in;
    endcase
end

// Data output are we using the MEM stage



always @* begin
    case (config_cache_swap)
        1'b1    : i_mem_data_cache_q <= o_instrution_data;
        default : i_mem_data_cache_q <= o_data_data;
    endcase
end

//------------------------------------------------------------------------------

wire [3:0] ram_fifo_usedw;

//{ [68] 1'b is_write, [67:64] masking, [63:32] address, [31:0] Data) }

model_fifo #(
    .width          (69),
    .widthu         (8)
)
ram_fifo_inst(
    .clk            (clk),
    .rst_n          (rst_n),
    
    .sclr           (`FALSE),
    
    .rdreq          (i_ram_fifo_rdreq),   //input
    .wrreq          (i_ram_fifo_wrreq),   //input
    .data           (i_ram_fifo_data),    //input [66:0]
    
    .empty          (o_ram_fifo_empty),   //output
    .full           (o_ram_fifo_full),    //output
    .q              (o_ram_fifo_q),       //output [66:0]
    .usedw          (o_ram_fifo_usedw)    //output [3:0]
);


endmodule

module model_data_cache_memory(
    input                       clk,
    
    input       [7:0]          address_a,
    
    output reg  [31:0]         q_a,
    input       [3:0]          wren_a,
    input       [31:0]         data_a
); /* verilator public_module */


reg [31:0] mem [255:0];

integer k;
initial
begin
for (k = 0; k < 256; k = k + 1)
begin
    mem[k] = 0;
end
end

always @(posedge clk) begin
    if(wren_a[0]) mem[address_a][ 7: 0] <= data_a[ 7: 0];
    if(wren_a[1]) mem[address_a][15: 8] <= data_a[15: 8];
    if(wren_a[2]) mem[address_a][23:16] <= data_a[23:16];
    if(wren_a[3]) mem[address_a][31:24] <= data_a[31:24];
    q_a <= mem[address_a];
end

endmodule

module cache_true_dual_ram(
    input                       clk,
    
    input       [widthad-1:0]   address_a,
    input                       wren_a,
    input       [width-1:0]     data_a,
    output reg  [width-1:0]     q_a,
    
    input       [widthad-1:0]   address_b,
    input                       wren_b,
    input       [width-1:0]     data_b,
    output reg  [width-1:0]     q_b
); /* verilator public_module */

parameter width     = 1;
parameter widthad   = 1;

reg [width-1:0] mem [(2**widthad)-1:0];

integer k;
initial
begin
for (k = 0; k < (2**widthad); k = k + 1)
begin
    mem[k] = 0;
end
end

always @(posedge clk) begin
    if(wren_a) mem[address_a] <= data_a;
    
    q_a <= mem[address_a];
end

always @(posedge clk) begin
    if(wren_b) mem[address_b] <= data_b;
    
    q_b <= mem[address_b];
end

endmodule
