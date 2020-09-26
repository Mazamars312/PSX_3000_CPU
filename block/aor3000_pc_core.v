`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.07.2020 10:24:14
// Design Name: 
// Module Name: aor3000_pc_core
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


module aor3000_pc_core(
    input               clk,
    input               rst_n,
    
    input               i_PC_update,
    input [31:0]        i_Branch_address,
    input               i_Branch_update,
    input [31:0]        i_Exception_address,
    input               i_Exception_update,
    
    output reg [31:0]   o_PC_counter,
    output reg [31:0]   o_RF_counter,
    output reg          o_pc_kill_instuction    
    );
    
    reg [31:0]          branch_address_waiting;
    reg                 branch_wating;
    reg                 i_Exception_wait;
    
    always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            o_PC_counter <= 32'hBFC00000;
            o_RF_counter <= 32'b0;
        end
        else if (i_PC_update) begin
            casez ({i_Exception_update, i_Branch_update, branch_wating})
                3'b1zz  :   o_PC_counter <= i_Exception_address;
                3'b01z  :   o_PC_counter <= i_Branch_address;
                3'b001  :   o_PC_counter <= branch_address_waiting;
                default :   o_PC_counter <= o_PC_counter + 4;
            endcase
            casez ({i_Exception_update, i_Branch_update, branch_wating})
                3'b1zz  :   o_RF_counter <= i_Exception_address;
                3'b01z  :   o_RF_counter <= i_Branch_address;
                3'b001  :   o_RF_counter <= branch_address_waiting;
                default :   o_RF_counter <= o_PC_counter;
            endcase
//            o_RF_counter <= o_PC_counter;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            branch_address_waiting <= 'b0;
            branch_wating <= 'b0;
            i_Exception_wait <= 'b0;
            o_pc_kill_instuction <= 'b0;
        end
        else begin
            casez ({i_Exception_update, i_Exception_wait, i_Branch_update , i_PC_update})
                4'b10z1 : begin      // exception happens and the branch is removed
                                branch_address_waiting <= 'b0;
                                branch_wating <= 'b0;
                                i_Exception_wait <= 'b0;
                                o_pc_kill_instuction <= 'b1;
                            end
                4'b10z0 : begin      // exception happens and the branch is removed
                                branch_address_waiting <= 'b0;
                                branch_wating <= 'b0;
                                o_pc_kill_instuction <= 'b0;
                                i_Exception_wait <= 'b1;
                            end
                
                4'b0011 : begin      // this is a successful branch and an update at the same time - cache hits can make this happen
                                branch_address_waiting <= 'b0;
                                branch_wating <= 'b0;
                                o_pc_kill_instuction <= 'b1;
                                i_Exception_wait <= 'b0;
                            end
                4'b0010 : begin      // this is when a waiting branch is in the counter as we are waiting for the next instruction to happen
                                branch_address_waiting <= i_Branch_address;
                                branch_wating <= 'b1;
                                o_pc_kill_instuction <= 'b0;
                                i_Exception_wait <= 'b0;
                            end
                4'b0001 : begin      // the update PC counter without a branch happening or this clears the second brank waiting as well
                                branch_address_waiting <= 'b0;
                                branch_wating <= 'b0;
                                o_pc_kill_instuction <= 'b0;
                                i_Exception_wait <= 'b0;
                            end
                default : begin
                                branch_address_waiting  <= branch_address_waiting;
                                branch_wating           <= branch_wating;
                                o_pc_kill_instuction    <= o_pc_kill_instuction;
                                i_Exception_wait        <= i_Exception_wait;
                            end
            endcase
        end
    end
    
endmodule
