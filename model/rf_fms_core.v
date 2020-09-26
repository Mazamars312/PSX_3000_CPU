`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.07.2020 18:27:12
// Design Name: 
// Module Name: rf_fms_core
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


module rf_fms_core (
    input           clk,
    input           rst_n,
    
    input           rf_valid,
    input           rf_load,
    output  reg     rf_pc_update,
    output  reg     rf_clear,
    output  reg     rf_stall,
    output  reg     rf_recover,
    
    input           rf_exe_multi_div,
    input           rf_exe_cop2_inst,
    
    input           exe_busy,
    input           exe_COP2_busy,
    
    input           exe_branch_start,
    input           exe_branch_waiting,
    
    input           mem_stall,
    
    input           mem_exception_start
);

    reg [1:0]   rf_state, rf_state_c;
    reg         exception_wait, exception_wait_c;
//    reg         instuction_waiting;
    reg         recover_instr, recover_instr_c;
    
    reg         rf_running;
    reg         rf_back_stalled;
    reg         rf_waiting_instru;

    parameter   start_up        =   2'd0;
    parameter   running         =   2'd1;
    parameter   waiting_instr   =   2'd2;
    parameter   backend_stall   =   2'd3;
    
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rf_state            <= start_up;
            recover_instr       <= 'b0;
            exception_wait      <= 'b0;
        end
        else begin
            rf_state <= rf_state_c;
            recover_instr       <= recover_instr_c;
            exception_wait   <= exception_wait_c;
        end
    end
    
always @* begin
    casez ({rf_pc_update,rf_valid,rf_back_stalled, rf_waiting_instru })
        4'b1zzz : recover_instr_c   <= 1'b0;
        4'b011z : recover_instr_c   <= 1'b1;
        4'b0101 : recover_instr_c   <= 1'b1;
        default : recover_instr_c <= recover_instr;
    endcase
    end

always @* begin
    casez ({rf_pc_update,mem_exception_start,rf_back_stalled, rf_waiting_instru })
        4'b1zzz : exception_wait_c   <= 1'b0;
        4'b011z : exception_wait_c   <= 1'b1;
        4'b0101 : exception_wait_c   <= 1'b1;
        default : exception_wait_c <= exception_wait;
    endcase
    end
    
    wire    valid_or_hold = rf_valid || recover_instr;
    
    always @* begin
        case (rf_state)
            running : begin // this is used for running cache system or a waiting instruction
                            // an exception will cause the RF stage to cancel that instrcution from passing on
                rf_running          <= 'b1;
                rf_back_stalled     <= 'b0;
                rf_waiting_instru   <= 'b0;
                if (rf_valid) begin
                    if (rf_load) rf_state_c <= backend_stall;
                    else rf_state_c <= running;
                end
                else if (|{mem_stall,                   // for a load we want to stall the next pipeline
                         (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                          rf_exe_cop2_inst && exe_COP2_busy})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                rf_state_c <= backend_stall;
                else rf_state_c <= waiting_instr;
            end
            waiting_instr : begin   // this is the core waiting for a instruction, If a exception happens then the the current wait 
                                    // will go back to a running state but the exception will cancel that instruction in the RF stage
                rf_running          <= 1'b0;
                rf_back_stalled     <= 1'b0;
                rf_waiting_instru   <= 1'b1;
                if (rf_valid) begin
                    if (|{rf_load, mem_stall,                   // for a load we want to stall the next pipeline
                         (rf_exe_multi_div && exe_busy && valid_or_hold),        // this is for a exe stall and the next is a exe busy call
                          rf_exe_cop2_inst && exe_COP2_busy && valid_or_hold})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                rf_state_c <= backend_stall;
                    else rf_state_c <= running;
                end
                else rf_state_c <= waiting_instr;
            end
            backend_stall : begin   // If there is a stall on the COP2, MEM or Multi/div cores and there is another instrcution for them
                                    // this state will be used to hold up the next instruction in the RF stage.
                                    // exceptions will cause the backend_stall to hold untill all is done and then update the PC counter
                rf_running          <= 1'b0;
                rf_back_stalled     <= 1'b1;
                rf_waiting_instru   <= 1'b0;
                if (|{ mem_stall,                       // for a load we want to stall the next pipeline
                      (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                       rf_exe_cop2_inst && exe_COP2_busy})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                rf_state_c <= backend_stall;
                else rf_state_c <= running;
            end
            default : begin // this is our start state, just a double check that there are no instant instructions there in cache ;-)
                rf_running          <= 'b0;
                rf_back_stalled     <= 'b0;
                rf_waiting_instru   <= 'b0;
                if (rst_n) rf_state_c <= running;
                else rf_state_c <= start_up;
            end
        endcase
    end

    always @* begin
        case (rf_state)
            running : begin
                if (|{rf_load, mem_stall,                       // for a load we want to stall the next pipeline
                         (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                          rf_exe_cop2_inst && exe_COP2_busy})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                            rf_pc_update <= 'b0;
                else if (rf_valid)          rf_pc_update <= 'b1;
                else                        rf_pc_update <= 'b0;
            end
            waiting_instr : begin
                if (|{rf_load, mem_stall,                       // for a load we want to stall the next pipeline
                         (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                          rf_exe_cop2_inst && exe_COP2_busy} && rf_valid)   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                            rf_pc_update <= 'b0;
                else if (rf_valid)          rf_pc_update <= 'b1;
                else                        rf_pc_update <= 'b0;
            end
            backend_stall : begin 
                if (|{mem_stall,                                // for a load we want to stall the next pipeline
                     (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                      rf_exe_cop2_inst && exe_COP2_busy})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                            rf_pc_update <= 'b0;
                else if (rf_valid)
                                            rf_pc_update <= 'b0;
                else                        rf_pc_update <= 'b1;
            end
            default : begin
                  if(rst_n)                 rf_pc_update <= 'b1;
                  else                      rf_pc_update <= 'b0;
            end
        endcase
    end
    
    always @* begin
        case (rf_state)
            running,
            waiting_instr,
            backend_stall : begin 
                if (|{mem_stall,                                // for a load we want to stall the next pipeline
                     (rf_exe_multi_div && exe_busy),        // this is for a exe stall and the next is a exe busy call
                     (rf_exe_cop2_inst && exe_COP2_busy)})   // this is for a COP2 stall and the next is a COP2 busy call or load/store
                                            rf_stall <= 'b1;
                else                        rf_stall <= 'b0;
            end
            default : begin
                                            rf_stall <= 'b0;
            end
        endcase
    end
    
    always @* begin
        case (rf_state)
            backend_stall : begin 
                if (rf_valid)               rf_recover <= 'b0;
                else if (recover_instr)     rf_recover <= 'b1;
                else                        rf_recover <= 'b0;
            end
            default : begin
                                            rf_recover <= 'b0;
            end
        endcase
    end
    
    always @* begin
        case (rf_state)
            backend_stall : begin 
                if (rf_valid && exception_wait)                         rf_clear <= 'b1;
                else if (recover_instr && exception_wait)               rf_clear <= 'b1;
                else                                                    rf_clear <= 'b0;
            end
            waiting_instr : begin 
                if (rf_valid && |{mem_exception_start,exception_wait})  rf_clear <= 'b1;
                else                                                    rf_clear <= 'b0;
            end
            default : begin
                if (rf_valid && mem_exception_start)                    rf_clear <= 'b1;
                else                                                    rf_clear <= 'b0;
            end
        endcase
    end

endmodule

