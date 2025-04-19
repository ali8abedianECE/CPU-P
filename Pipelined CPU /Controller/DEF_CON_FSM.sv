`define RESET 7'd0
// `define State_IF_1 7'd1
// `define State_IF_2 7'd2
// `define State_Update_PC 7'd3
// `define State_Decode 7'd4

//PIPELINE STAGES FOR THE FSM. 
`define Stage_IF   7'd0   // Instruction Fetch, update program counter by 1 and only 1. 
`define Stage_ID   7'd1   // Instruction Decode, get the operands and stuff so after the memory fetch stage. 
`define Stage_EX   7'd2   // Execute DO instruction
`define Stage_MEM  7'd3   // Memory Access write to memeory if needed. 
`define Stage_WB   7'd4   // Write Back to regfile. 

//THESE INSTRUCTIONS BELLOW MOST ARE INVALID WITH THE PIPLINE TIME TO DO SOMEINTEGERATION. 
`define MOV_imm8 7'd1
`define MOV_Rd_Rm 7'd2
`define ADD_Rd_Rn_Rm 7'd3
`define CMP_Rn_Rm 7'd4
`define AND_Rd_Rn_Rm 7'd5
`define MVN_Rd_Rm 7'd6
`define LDR_RD_RN_IMM5 7'd7
`define STR_RD_RN_IMM5 7'd8
`define HALTED 7'd9

`define Branch_B 7'd10
`define Branch_EQ 7'd11
`define Branch_NE 7'd12
`define Branch_LT 7'd13
`define Branch_LE 7'd14
`define Branch_BL 7'd15
`define Branch_BX 7'd16
`define Branch_BLX 7'd17

`define State_Branch_Update_PC_2 7'd45 
`define State_Branch_Update_PC_0 7'd46 //STUPID STATE

module DEF_CON_FSM(Reset, op, opCode, cond, State);
    input Reset;
    input [1:0] op;
    input [2:0] opCode, cond;
    output load_pc, load_ir; 
    output wire [6:0] State;

    reg [6:0] State_Transitions; 

    assign State = State_Transitions;

    always_comb begin 
        if(Reset) begin 
            State_Transitions = `RESET;
        end else begin 
            casex({opCode, op, cond})
                8'b11010xxx: State_Transitions = `MOV_imm8;
                8'b11000xxx: State_Transitions = `MOV_Rd_Rm;
                8'b10100xxx: State_Transitions = `ADD_Rd_Rn_Rm;
                8'b10101xxx: State_Transitions = `CMP_Rn_Rm;
                8'b10110xxx: State_Transitions = `AND_Rd_Rn_Rm;
                8'b10111xxx: State_Transitions = `MVN_Rd_Rm;
                8'b01100xxx: State_Transitions = `LDR_RD_RN_IMM5;
                8'b10000xxx: State_Transitions = `STR_RD_RN_IMM5;
                8'b111xxxxx: State_Transitions = `HALTED;
                8'b00100000: State_Transitions = `Branch_B;
                8'b00100001: State_Transitions = `Branch_EQ;
                8'b00100010: State_Transitions = `Branch_NE;
                8'b00100011: State_Transitions = `Branch_LT;
                8'b00100100: State_Transitions = `Branch_LE;
                8'b01011111: State_Transitions = `Branch_BL;
                8'b01000xxx: State_Transitions = `Branch_BX;
                8'b01010111: State_Transitions = `Branch_BLX;
                default: State_Transitions = {7{1'bx}};//here to prevent a latch. 
            endcase
        end 
    end 


    //     end else begin 
    //         case(State_Transitions)
    //             `State_MOV_imm8: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_MOV_Rd_Rm_load_Rm_B: begin 
    //                 State_Transitions <= `State_MOV_Rd_Rm_Prefrom_Shift_load_C;
    //             end 
    //             `State_MOV_Rd_Rm_Prefrom_Shift_load_C: begin 
    //                 State_Transitions <= `State_MOV_Rd_Rm_write_Rd;
    //             end
    //             `State_MOV_Rd_Rm_write_Rd: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_ADD_Rd_Rn_Rm_load_Rm_B: begin 
    //                 State_Transitions <= `State_ADD_Rd_Rn_Rm_load_Rn_A;
    //             end
    //             `State_ADD_Rd_Rn_Rm_load_Rn_A: begin 
    //                 State_Transitions <= `State_ADD_Rd_Rn_Rm_Add_Shift_load_C;
    //             end
    //             `State_ADD_Rd_Rn_Rm_Add_Shift_load_C: begin 
    //                 State_Transitions <= `State_ADD_Rd_Rn_Rm_write_Rd;
    //             end
    //             `State_ADD_Rd_Rn_Rm_write_Rd: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_CMP_Rn_Rm_load_Rm_B: begin 
    //                 State_Transitions <= `State_CMP_Rn_Rm_load_Rn_A;
    //             end
    //             `State_CMP_Rn_Rm_load_Rn_A: begin 
    //                 State_Transitions <= `State_CMP_Rn_Rm_load_flags;
    //             end
    //             `State_CMP_Rn_Rm_load_flags: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_AND_Rd_Rn_Rm_load_Rm_B: begin 
    //                 State_Transitions <= `State_AND_Rd_Rn_Rm_load_Rn_A;
    //             end
    //             `State_AND_Rd_Rn_Rm_load_Rn_A: begin 
    //                 State_Transitions <= `State_AND_Rd_Rn_Rm_And_Shift_load_C;
    //             end
    //             `State_AND_Rd_Rn_Rm_And_Shift_load_C: begin 
    //                 State_Transitions <= `State_AND_Rd_Rn_Rm_write_Rd;
    //             end
    //             `State_AND_Rd_Rn_Rm_write_Rd: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_MVN_Rd_Rm_load_Rm_B: begin 
    //                 State_Transitions <= `State_MVN_Rd_Rm_Perform_Shift_load_C;
    //             end
    //             `State_MVN_Rd_Rm_Perform_Shift_load_C: begin 
    //                 State_Transitions <= `State_MVN_Rd_Rm_write_Rd;
    //             end
    //             `State_MVN_Rd_Rm_write_Rd: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_LDR_RD_RN_IMM5_LOAD_A_AND_BSEL: begin 
    //                 State_Transitions <= `State_LDR_RD_RN_IMM5_LOAD_C;
    //             end
    //             `State_LDR_RD_RN_IMM5_LOAD_C: begin 
    //                 State_Transitions <= `State_LDR_RD_RN_IMM5_LOAD_INTO_DATA_ADDRESS_REG;
    //             end
    //             `State_LDR_RD_RN_IMM5_LOAD_INTO_DATA_ADDRESS_REG: begin 
    //                 State_Transitions <= `State_LDR_RD_RN_IMM5_READ_FROM_MEM;
    //             end
    //             `State_LDR_RD_RN_IMM5_READ_FROM_MEM: begin 
    //                 State_Transitions <= `State_LDR_RD_RN_IMM5_LOAD_RD_WITH_MDATA;
    //             end
    //             `State_LDR_RD_RN_IMM5_LOAD_RD_WITH_MDATA: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_STR_RD_RN_IMM5_LOAD_A_AND_BSEL: begin 
    //                 State_Transitions <= `State_STR_RD_RN_IMM5_LOAD_C;
    //             end
    //             `State_STR_RD_RN_IMM5_LOAD_C: begin 
    //                 State_Transitions <= `State_STR_RD_RN_IMM5_LOAD_INTO_DATA_ADDRESS_REG;
    //             end
    //             `State_STR_RD_RN_IMM5_LOAD_INTO_DATA_ADDRESS_REG: begin 
    //                 State_Transitions <= `State_STR_RD_RN_IMM5_LOAD_B_ASEL_1;
    //             end
    //             `State_STR_RD_RN_IMM5_LOAD_B_ASEL_1: begin 
    //                 State_Transitions <= `State_STR_RD_RN_IMM5_LOAD_C_2;
    //             end
    //             `State_STR_RD_RN_IMM5_LOAD_C_2: begin 
    //                 State_Transitions <= `State_STR_RD_RN_IMM5_WRITE_TO_MEM;
    //             end
    //             `State_STR_RD_RN_IMM5_WRITE_TO_MEM: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_HALTED: begin 
    //                 State_Transitions <= `State_HALTED;
    //             end
    //             //TODO FIX logic here for branching.
    //             `State_Branch_EQ: begin 
    //                 if (Z === 1'b1) begin 
    //                    State_Transitions <= `State_Branch_Update_PC_2;
    //                 end else begin 
    //                     State_Transitions <= `State_IF_1;
    //                 end
    //             end
    //             `State_Branch_NE: begin 
    //                 if (Z === 1'b0) begin 
    //                     State_Transitions <= `State_Branch_Update_PC_2;
    //                 end else begin 
    //                     State_Transitions <= `State_IF_1;
    //                 end
    //             end
    //             `State_Branch_LT: begin 
    //                 if (N !== V) begin 
    //                     State_Transitions <= `State_Branch_Update_PC_2;
    //                 end else begin 
    //                     State_Transitions <= `State_IF_1;
    //                 end
    //             end
    //             `State_Branch_LE: begin 
    //                 if (Z === 1'b1 | N !== V) begin 
    //                     State_Transitions <= `State_Branch_Update_PC_2;
    //                 end else begin 
    //                     State_Transitions <= `State_IF_1;
    //                 end
    //             end
    //             `State_Branch_B: begin 
    //                 State_Transitions <= `State_Branch_Update_PC_2;
    //             end
    //             `State_Branch_Update_PC_0: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end 
    //             `State_Branch_Update_PC_2: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             //TODO FIX logic here for branching.
    //             `State_Branch_BL_R7: begin 
    //                 State_Transitions <= `State_Branch_Update_PC_2;
    //             end
    //             `State_Branch_BX: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             `State_Branch_BLX_R7: begin 
    //                 State_Transitions <= `State_Branch_BLX_PC;
    //             end
    //             `State_Branch_BLX_PC: begin 
    //                 State_Transitions <= `State_IF_1;
    //             end
    //             default: begin 
    //                 State_Transitions <= `State_IF_1;  
    //             end
    //         endcase
    //     end
    // end

endmodule : DEF_CON_FSM