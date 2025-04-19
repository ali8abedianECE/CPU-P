`define MEM_NONE 2'b00
`define MEM_READ 2'b01
`define MEM_WRITE 2'b10
module datapath(clk, reset, enablePC, Zout, Nout, Vout, OUT, HALT, PCFinal, MEM_CMD, dinRam, ReadMEM, PCCCC);

    input [15:0] ReadMEM; 
    input clk, reset, enablePC;

    output wire Zout, Nout, Vout, HALT;
    output wire [8:0] PCFinal; 
    output wire [1:0] MEM_CMD;
    output wire [15:0] dinRam;  
    output wire [15:0] OUT;
    output reg [8:0] PCCCC;

    wire imJumpFlag; 
    wire [8:0] imJump;
    wire Flush1, Flush2, Flush3; 
    wire [8:0] PC;
    wire [8:0] PCPP; 
    wire [2:0] ZflagsP;
    wire [15:0] ALUoutP;  
    wire [2:0] Zflags;
    wire [15:0] disconnect; 
    //wire [15:0] ReadMEM; 
    wire [15:0] data_in_to_regs;
    wire [15:0] Rn, Rd, Rm;
    wire [2:0] Rd_NUMP, Rn_NUMP, Rm_NUMP, opcodeP, condP;
    wire [1:0] opP, ALUopP, shiftP;
    wire [15:0] sximm5P, sximm8P, data_inP;
    wire [8:0] PCP;  
    wire [2:0] Rd_NUM, Rn_NUM, Rm_NUM, opcode, cond, opcodePP;
    wire [1:0] op, ALUop, shift;
    wire [15:0] sximm5, sximm8, data_in;
    wire [8:0] PCprop; 
    wire [15:0] INSTR, INSTRD;
    wire [8:0] address; 
    wire [15:0] outDEC; 
    wire [15:0] Bin, Ain, ALUOUT, Bin_BEF; //Correct
    wire Stall1, Stall2, Stall3, load_PC; 
    wire ReleaseStall, isStall, isStallP, ReleaseStallP; 
    wire [1:0] WRITE_MEM; 
    wire load_ir;
    wire REGWRITE, SelectAIN, SelectBIn, loads, VSEL;
    wire [1:0] VSEL2, VSEL2P;
    wire REGWRITEP, SelectAINP, SelectBInP, loadsP, loadsPP, isLDR, REGWRITELDR;
    wire StallEX, isHALT, isHALTP;
    wire [8:0] PCCC, PCpropPP, PCtoProp, PCPS;
    reg resetMEM; 
    wire Z, N, V, ZI, NI, VI, branch_taken, is_str; 

    //PROGRAMCOUNTER PIPELINE
    ProgramCounter PC1(.PCout(PC), .clk(clk), .reset(reset || resetMEM), .enablePC(enablePC), .imJump(imJump), .imJumpFlag(imJumpFlag), .load_PC(load_PC && ~(is_str || isLDR)));
    //assign PCCCC =  PCP; 
    always_comb begin 
        if(reset || resetMEM) begin 
            PCCCC = 9'd0;
        end else begin
            PCCCC = PCP;
        end
    end
    PIPELINE_PCIF PCIFPIPE(.PC(PC), .address(address), .Flush1(Flush1  || reset || resetMEM), .Stall1(Stall1 || is_str || isLDR), .clk(clk)); //LOOK AT FLUSH1, and STALL1, 
    RAM2 #(16, 8) READ_ONLY_MEM(.clk(clk), .read_address(address[7:0]), .write_address(address[7:0]), .write(1'b0), .din(16'd0), .dout(INSTR)); 

    Reg #(9) PCPIPELINE(.in(address), .out(PCtoProp), .clk(clk), .load(~(Flush1 || Stall1 || is_str || isLDR)  || reset));

    PIPELINE_IFID IFIDPIPE(.PCG(PCtoProp), .PCOUT(PCprop), .INSTRUCTION_READ(INSTR), .INSTRUCTION_DECODE(INSTRD), .Flush2(Flush2 || reset || resetMEM), .Stall2(Stall2), .clk(clk)); //TODO LOOK AT FLUSH2, and STALL2.

    Reg instructionRegister(.in(INSTRD), .load(load_ir & ~Flush2 ), .clk(clk), .out(outDEC));

    Reg #(9) PCPIPELbINE(.in(PCprop), .out(PCPS), .clk(clk), .load((load_ir & ~Flush2)  || reset || resetMEM));

    InstaDecoder INSTR_DECODE(.instructionReg(outDEC), .opcode(opcode), .op(op), .ALUop(ALUop), .sximm5(sximm5), 
                              .sximm8(sximm8), .shift(shift), .Rd(Rd_NUM), .Rn(Rn_NUM), .Rm(Rm_NUM), .cond(cond));
    //should_Stall SSTDET(.op(op), .opcode(opcode), .Stall1(Stall1), .Stall2(Stall2), .load_PC(load_PC), .ReleaseStall(ReleaseStallP), .isStall(isStall), .clk(clk));
    //PIPELINEREGISTER #(1) STALLSIG(.indata(isStall), .outdata(isStallP), .clk(clk));
    //StallCounter SSTCC(.isStall(isStallP), .Stall3(Stall3), .ReleaseStall(ReleaseStall), .clk(clk));
    //PIPELINEREGISTER #(1) STALLREL(.indata(ReleaseStall), .outdata(ReleaseStallP), .clk(clk));
    wire [2:0] state;

    stall_module SST(.op(op), .opcode(opcode), .Stall1(Stall1), .Stall2(Stall2), .Stall3(Stall3), .load_PC(load_PC), .clk(clk), .WRITE_MEM(WRITE_MEM), .load_ir(load_ir), .isLDR(isLDR), .is_str(is_str), .REGWRITELDR(REGWRITELDR), .state(state));
    Controller CCT(.opcode(opcode), .op(op), .cond(cond), .REGWRITE(REGWRITE), .VSEL(VSEL), .VSEL2(VSEL2), .SelectAIN(SelectAIN), .SelectBIn(SelectBIn), .loads(loads));
    MUX2a #(16) MUXD(.out(data_in), .input0({7'b0, PCPS} + 9'd1), .input1(sximm8), .select(VSEL)); //TODO add VSEL to CONTROLLER. 
    Reg #(9) NZIA(.in(PCprop), .clk(clk), .out(PCpropPP), .load(load_PC));
    PIPELINE_IDEX IDEXPIPE(.Rd(Rd_NUM),  .Rn(Rn_NUM),  .Rm(Rm_NUM),  .opcode(opcode),  .op(op),  .cond(cond),  .ALUop(ALUop),  .shift(shift),  .sximm8(sximm8),  .sximm5(sximm5),  .data_in(data_in), .PC(PCPS), .REGWRITE(REGWRITE), .VSEL2(VSEL2), .SelectAIN(SelectAIN), .SelectBIn(SelectBIn), .loads(loads),
                           .RdP(Rd_NUMP), .RnP(Rn_NUMP), .RmP(Rm_NUMP), .opcodeP(opcodeP), .opP(opP), .condP(condP), .ALUopP(ALUopP), .shiftP(shiftP), .sximm8P(sximm8P), .sximm5P(sximm5P), .data_inP(data_inP), .PCP(PCP), .REGWRITEP(REGWRITEP), .VSEL2P(VSEL2P), .SelectAINP(SelectAINP), .SelectBInP(SelectBInP), .loadsP(loadsP),
                           .Flush3(Flush3 || reset || resetMEM), .Stall3(Stall3 || branch_taken || StallEX), .clk(clk));  //TODO look at Flush and Stall.   

    isHalted HALTS(.opcode(opcodeP), .isHALT(isHALT));
    ///isHalted HATLED2(.opcode(opcodePP), .isHALT(HALT));
    wire current_halt;
    reg halt_detected;
    isHalted HALT_CHECK(.opcode(opcodeP), .isHALT(current_halt));


    always_ff @(posedge clk) begin 
        resetMEM <= reset;
    end

    // Latch HALT once detected
    always_ff @(posedge clk) begin
        if (reset)
            halt_detected <= 1'b0;
        else if (current_halt)
            halt_detected <= 1'b1;
        else 
            halt_detected <= 1'b0;
    end

    // HALT is active if we've ever detected a HALT instruction
    assign HALT = halt_detected || current_halt;

    MUX3a #(16) EXECUTEMOVEIMM(.out(data_in_to_regs), .in0(ReadMEM), .in1(data_inP), .in2(ALUOUT), .select(VSEL2P)); //TODO determine VSEL2
    //wire regwrite; 
    //assign regwrite = isLDR ? REGWRITELDR : REGWRITEP; //TODO CHANGE THIS PART
    wire memory_active, final_regwrite; 
    assign memory_active = (state != 3'd0);
    assign final_regwrite = memory_active ? REGWRITELDR : REGWRITEP; 
    wire [2:0] actual_RD, actual_RDREAD; 
    MUX2a #(3) SPECIALCASEMUX1(.out(actual_RD), .input0(Rd_NUMP), .input1(Rn_NUMP), .select(({opcodeP, opP} == 5'b01010))); 
    MUX2a #(3) SPECIALCASEMUX2(.out(actual_RDREAD), .input0(Rd_NUMP), .input1(Rn_NUMP), .select(~({opcodeP, opP} == 5'b01010))); 
    wire [15:0] RdREAD; 
    regfile REGFILE(.data_in(data_in_to_regs), .RnNUM(Rn_NUMP), .RmNUM(Rm_NUMP), .RdNUM(actual_RD), .RdNUMREAD(actual_RDREAD), .write(final_regwrite), .writenum(actual_RD), .clk(clk), .Rn(Rn), .Rm(Rm), .Rd(Rd), .RdREAD(RdREAD));
    MUX2a #(16) MuxALUAIN(.out(Ain), .input0(Rn), .input1(16'd0), .select(SelectAINP));
    MUX2a #(16) MuxALUBIN(.out(Bin), .input0(Bin_BEF), .input1(sximm5P), .select(SelectBInP));
    shifter SHIFTER(.in(Rm), .shift(shiftP), .sout(Bin_BEF));
    ALU ALU1(.Ain(Ain), .Bin(Bin), .ALUop(ALUopP), .out(ALUOUT), .Z(Zflags));

   // PIPELINEREGISTER PCPPj(.indata(PC), .outdata(PCCC), .clk(clk));

    Reg #(1) registerStatusI(.in(ZflagsP[0]), .load(loadsP), .clk(clk), .out(ZI));
    Reg #(1) registerStatus1I(.in(ZflagsP[1]), .load(loadsP), .clk(clk), .out(NI));
    Reg #(1) registerStatus2I(.in(ZflagsP[2]), .load(loadsP), .clk(clk), .out(VI));

    BranchController BCCT(.opcode(opcodeP), .op(opP), .cond(condP), .PC(PCP), .sximm8(sximm8P), .imJump(imJump), .imJumpFlag(imJumpFlag), .Flush1(Flush1), .Flush2(Flush2), .Flush3(Flush3), .Z(Z), .N(N), .V(V), .clk(clk), .reset(reset), .StallEX(StallEX), .branch_taken(branch_taken), .Rd(Rd), .Rdspecial(RdREAD)); 
 //   RAM #(16, 8) MEM(.clk(clk), .read_address(ALUOUT[7:0]), .write_address(ALUOUT[7:0]), .write(WRITE_MEM == `MEM_WRITE), .din(Rd), .dout(ReadMEM));

    PIPELINE_EXFI EXFIPIPE(.ALUout(ALUOUT), .Zflags(Zflags), .PC(PCP), .loadsP(loadsP), .opcodeP(opcodeP), .opcodePP(opcodePP), .loadsPP(loadsPP), .ALUoutP(ALUoutP), .ZflagsP(ZflagsP), .PCP(PCFinal), .clk(clk));

    Reg #(1) registerStatus(.in(Zflags[0]), .load(loadsP), .clk(clk), .out(Z));
    Reg #(1) registerStatus1(.in(Zflags[1]), .load(loadsP), .clk(clk), .out(N));
    Reg #(1) registerStatus2(.in(Zflags[2]), .load(loadsP), .clk(clk), .out(V));

    assign Zout = Z; 
    assign Nout = N; 
    assign Vout = V;
    assign OUT = ALUOUT; 
    assign MEM_CMD = WRITE_MEM;
    assign dinRam = Rd; 


//     // Debug monitoring
//     always @(posedge clk) begin
//     $display("WRITE_MEM: %b, REGWRITELDR: %b. VSEL2P: %b ReadMEM: %h, data_in_to_regs: %h, REGWRITEP: %b, Stall1: %b,  Stall2: %b,  Stall3: %b", 
//              WRITE_MEM, REGWRITELDR, VSEL2P, ReadMEM, data_in_to_regs,REGWRITEP, Stall1, Stall2, Stall3);
//   end
endmodule : datapath

module isHalted(opcode, isHALT); 
    input [2:0] opcode; 
    output reg isHALT; 

    always_comb begin
        if(opcode == 3'b111) begin 
            isHALT = 1'b1; 
        end else begin 
            isHALT = 1'b0; 
        end
    end

endmodule : isHalted
// module BranchControll(opcode, op, cond, PC, sximm8, imJump, imJumpFlag); 

//     input [2:0] opcode, cond; 
//     input [1:0] op; 
//     input [8:0] PC; 
//     input [15:0] sximm8; 
//     input Z, N, V, clk, reset; 

//     output [8:0] imJump;

//     assign imJump = branch_taken ? PC + 9'd1 + sximm8[8:0]: PC; 

//     //Determines if a branch is taken. 
//     assign branch_taken = (
//         // Unconditional Branch (B)
//         ({opcode, op, cond} == 8'b00100000) ||

//         // BEQ (Branch if Equal)
//         ({opcode, op, cond} == 8'b00100001 && Z == 1'b1) ||

//         // BNE (Branch if Not Equal)
//         ({opcode, op, cond} == 8'b00100010 && Z == 1'b0) ||

//         // BLT (Branch if Less Than)
//         ({opcode, op, cond} == 8'b00100011 && N != V) ||

//         // BLE (Branch if Less or Equal)
//         ({opcode, op, cond} == 8'b00100100 && (N != V || Z == 1'b1))
//     );




// endmodule : BranchControll

`define State_Branch_Controller_IDLE 3'd0
`define State_Branch_Controller_Taken 3'd1
`define State_Branch_Controller_Flush1 3'd2
`define State_Branch_Controller_Flush2 3'd3
`define State_Branch_Controller_Flush3 3'd4

module BranchController(opcode, op, cond, PC, sximm8, imJump, imJumpFlag, Flush1, Flush2, Flush3, Z, N, V, clk, reset, StallEX, branch_taken, Rd, Rdspecial); 

    input [2:0] opcode, cond; 
    input [1:0] op; 
    input [8:0] PC; 
    input [15:0] sximm8, Rd, Rdspecial; 
    input Z, N, V, clk, reset; 
    wire branchBX; 
    wire SN; 

    output [8:0] imJump;
    assign imJump = branchBX ? Rd[8:0] : SN ? Rdspecial[8:0] : PC + sximm8[8:0] + 9'd1;

    output reg imJumpFlag; 
    output reg Flush1, Flush2, Flush3, StallEX; 

    reg [2:0] State;
    reg SpecialCaseTracker; 

    output branch_taken;



    assign SN = (({opcode, op} == 5'b01010));

    assign branchBX = (
        ({opcode, op} == 5'b01000)
    );


    assign branch_taken = (
        // Unconditional Branch (B)
        ({opcode, op, cond} == 8'b00100000) ||

        // BEQ (Branch if Equal)
        ({opcode, op, cond} == 8'b00100001 && Z == 1'b1) ||

        // BNE (Branch if Not Equal)
        ({opcode, op, cond} == 8'b00100010 && Z == 1'b0) ||

        // BLT (Branch if Less Than)
        ({opcode, op, cond} == 8'b00100011 && N != V) ||

        // BLE (Branch if Less or Equal)
        ({opcode, op, cond} == 8'b00100100 && (N != V || Z == 1'b1)) ||

        //BL (Branch Taken for return)
        ({opcode, op, cond} == 8'b01011111) ||

        ({opcode, op} == 5'b01000) ||

        ({opcode, op} == 5'b01011) || 

        ({opcode, op} == 5'b01010) 
    );


    always_ff @(posedge clk) begin 
        if (reset) begin
            State <= `State_Branch_Controller_IDLE;
            StallEX <= 1'b0; 
        end else if(State == `State_Branch_Controller_IDLE) begin 
            if(branchBX || SN) begin 
                State <= `State_Branch_Controller_Taken;
                StallEX <= 1'b1;
                SpecialCaseTracker <= branchBX;
            end else begin 
            case({opcode, op, cond})
                // 8'b111xxxxx: State_Transitions <= `State_HALTED;
                8'b00100000: begin 
                    State <= `State_Branch_Controller_Taken;
                    StallEX <= 1'b1;
                end
                8'b00100001: begin 
                    if(Z) begin 
                        State <= `State_Branch_Controller_Taken;
                        StallEX <= 1'b1; 
                    end else begin 
                        State <= `State_Branch_Controller_IDLE;
                        StallEX <= 1'b0;
                    end
                end
                8'b00100010: begin 
                    if(Z === 1'b0) begin 
                        State <= `State_Branch_Controller_Taken;
                        StallEX <= 1'b1;
                    end else begin 
                        State <= `State_Branch_Controller_IDLE;
                        StallEX <= 1'b0;
                    end
                end
                8'b00100011: begin 
                    if(N !== V) begin 
                        State <= `State_Branch_Controller_Taken;
                        StallEX <= 1'b1;
                    end else begin 
                        State <= `State_Branch_Controller_IDLE;
                        StallEX <= 1'b0;
                    end
                end
                8'b00100100: begin
                    if(N !== V || Z === 1'b1) begin 
                        State <= `State_Branch_Controller_Taken;
                        StallEX <= 1'b1; 
                    end else begin 
                        State <= `State_Branch_Controller_IDLE;
                        StallEX <= 1'b0;
                    end
                end
                8'b01011111: begin 
                    State <= `State_Branch_Controller_Taken;
                    StallEX <= 1'b1; 
                end
                // 8'b01000xxx: State_Transitions <= `State_Branch_BX;
                // 8'b01010111: State_Transitions <= `State_Branch_BLX_R7;
                default: begin 
                    State <= `State_Branch_Controller_IDLE; 
                    StallEX <= 1'b0;
                end
            endcase
            SpecialCaseTracker <= 1'b0; 
        end end else if(State == `State_Branch_Controller_Taken) begin 
            State <= `State_Branch_Controller_Flush1;
            StallEX <= 1'b1; 
            SpecialCaseTracker <= SpecialCaseTracker;
        end else if(State === `State_Branch_Controller_Flush1) begin 
            State <= `State_Branch_Controller_Flush2;
            StallEX <= 1'b1;
            SpecialCaseTracker <= SpecialCaseTracker;
        end else if(State === `State_Branch_Controller_Flush2) begin 
            State <= `State_Branch_Controller_Flush3;
            StallEX <= 1'b0;
            SpecialCaseTracker <= SpecialCaseTracker;
        end else if(State === `State_Branch_Controller_Flush3) begin 
            State <= `State_Branch_Controller_IDLE;
            StallEX <= 1'b0;
            SpecialCaseTracker <= SpecialCaseTracker;
        end else begin 
            State <= `State_Branch_Controller_IDLE;
            StallEX <= 1'b0; 
            SpecialCaseTracker <= 1'b0;
        end
    end

    always_comb begin 
        case(State) 
            `State_Branch_Controller_IDLE: begin 
                Flush1 = 1'b0; 
                Flush2 = 1'b0; 
                Flush3 = 1'b0; 
                imJumpFlag = 1'b0;  
            end
            `State_Branch_Controller_Taken: begin 
            // When branch is taken, flush IF/ID and update PC
            Flush1 = 1'b1;  // Flush IF 
            Flush2 = 1'b1;  // Flush ID
            Flush3 = 1'b1;  // Flush EX
            imJumpFlag = 1'b1;  // Update PC to branch target
        end
        `State_Branch_Controller_Flush1: begin 
            // First cycle after branch taken
            Flush1 = 1'b0;  // Allow new instruction fetch
            Flush2 = 1'b1;  // Keep ID flushed
            Flush3 = 1'b1;  // Keep EX flushed
            imJumpFlag = 1'b0;  // PC already updated
        end
        `State_Branch_Controller_Flush2: begin 
            // Second cycle after branch taken
            Flush1 = 1'b0;  // Normal fetch
            Flush2 = 1'b1;  // Allow ID
            Flush3 = 1'b1;  // Final flush of EX
            imJumpFlag = 1'b0;
        end
        `State_Branch_Controller_Flush3: begin //THIS PART IS THE PROBLEM TODO TMRW FLUSH3 = 0 but not sure if it does anything bad. 
            // Return to normal operation
            Flush1 = 1'b0;
            Flush2 = 1'b0;
            Flush3 = 1'b1;
            imJumpFlag = 1'b0;
        end
        default: begin 
            Flush1 = 1'b0;
            Flush2 = 1'b0;
            Flush3 = 1'b0;
            imJumpFlag = 1'b0;
        end
        endcase
    end

endmodule : BranchController


module Controller(opcode, op, cond, REGWRITE, VSEL, VSEL2, SelectAIN, SelectBIn, loads); 
    input [2:0] opcode, cond; 
    input [1:0] op; 

    output reg REGWRITE, VSEL, SelectAIN, SelectBIn, loads; 
    output reg [1:0] VSEL2; 

    always_comb begin 
        casex({opcode, op, cond}) 
            8'b11010xxx: begin 
                REGWRITE = 1'b1; 
                VSEL = 1'b1; 
                VSEL2 = 2'd1; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
            8'b11000xxx: begin 
                //State_Transitions <= `State_MOV_Rd_Rm_load_Rm_B;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd2; 
                SelectAIN = 1'd1;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
            8'b10100xxx: begin 
                // State_Transitions <= `State_ADD_Rd_Rn_Rm_load_Rm_B;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd2; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
            8'b10101xxx: begin 
                //State_Transitions <= `State_CMP_Rn_Rm_load_Rm_B;
                REGWRITE = 1'b0; 
                VSEL = 1'b0; 
                VSEL2 = 2'd2; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd0;
                loads = 1'b1;
            end
            8'b10110xxx: begin 
                //State_Transitions <= `State_AND_Rd_Rn_Rm_load_Rm_B;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd2; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
            8'b10111xxx: begin 
                //State_Transitions <= `State_MVN_Rd_Rm_load_Rm_B;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd2; 
                SelectAIN = 1'd1;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
            8'b01100xxx: begin 
                //State_Transitions <= `State_LDR_RD_RN_IMM5_LOAD_A_AND_BSEL;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd0; 
                SelectAIN = 1'b0;
                SelectBIn = 1'b1;
                loads = 1'b0;
            end
            8'b10000xxx: begin 
                //State_Transitions <= `State_STR_RD_RN_IMM5_LOAD_A_AND_BSEL;
                REGWRITE = 1'b0; 
                VSEL = 1'b0; 
                VSEL2 = 2'd0; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd1;
                loads = 1'b0;
            end
            // 8'b111xxxxx: State_Transitions <= `State_HALTED;
            // 8'b00100000: State_Transitions <= `State_Branch_B;
            // 8'b00100001: State_Transitions <= `State_Branch_EQ;
            // 8'b00100010: State_Transitions <= `State_Branch_NE;
            // 8'b00100011: State_Transitions <= `State_Branch_LT;
            // 8'b00100100: State_Transitions <= `State_Branch_LE;
            8'b01011111: begin 
                //State_Transitions <= `State_Branch_BL_R7;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd1; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd1;
                loads = 1'b0;
            end
            // 8'b01000xxx: State_Transitions <= `State_Branch_BX;
            8'b01010111: begin 
                //State_Transitions <= `State_Branch_BLX_R7;
                REGWRITE = 1'b1; 
                VSEL = 1'b0; 
                VSEL2 = 2'd1; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd1;
                loads = 1'b0;
            end
            default: begin 
                //State_Transitions <= {7{1'bx}};
                REGWRITE = 1'b0; 
                VSEL = 1'b0; 
                VSEL2 = 2'd0; 
                SelectAIN = 1'd0;
                SelectBIn = 1'd0;
                loads = 1'b0;
            end
        endcase
    end

endmodule : Controller

`define State_Stall_IDLE 3'd0
`define State_Stall_ACTIVE 3'd1

module stall_module(clk, op, opcode, Stall1, Stall2, Stall3, load_PC, WRITE_MEM, load_ir, isLDR, is_str, REGWRITELDR, state);
    input clk;
    input [1:0] op;
    input [2:0] opcode;
    output Stall1;
    output Stall2;
    output Stall3;
    output load_PC;
    output reg [1:0] WRITE_MEM;
    output load_ir; 
    output reg isLDR;
    output reg is_str;
    output reg REGWRITELDR;

    reg Stall1_CAL, Stall2_CAL, Stall3_CAL, load_PC_CAL, load_ir_CAL;
    output reg [2:0] state;
    wire is_mem_op; 

    assign Stall1 = Stall1_CAL;
    assign Stall2 = Stall2_CAL;
    assign Stall3 = Stall3_CAL;
    assign load_PC = load_PC_CAL;
    assign load_ir = load_ir_CAL; 

    // Detect memory operations (LDR or STR)
    assign is_mem_op = ({opcode, op} == 5'b01100 || {opcode, op} == 5'b10000);
    always_comb begin 
        if({opcode, op} == 5'b10000) begin 
            is_str = 1'b1;
            isLDR = 1'b0;
        end else if({opcode, op} == 5'b01100) begin 
            is_str = 1'b0;
            isLDR = 1'b1;
        end else begin 
            is_str = 1'b0;
            isLDR = 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        case(state)
            `State_Stall_IDLE: begin
                if (is_mem_op) begin
                    // When an LDR or STR is detected, activate stall for one cycle
                    state <= `State_Stall_ACTIVE;

                    // Stall the pipeline stages
                    Stall1_CAL <= 1'b1;
                    Stall2_CAL <= 1'b1;
                    Stall3_CAL <= 1'b1;

                    // Do not increment PC during stall
                    load_PC_CAL <= 1'b0;

                    // Do not load new instructions during stall
                    load_ir_CAL <= 1'b0;

                    // Set memory write enable accordingly
                    WRITE_MEM <= is_str ? `MEM_WRITE : `MEM_READ;

                    // Set REGWRITELDR for LDR
                    REGWRITELDR <= 1'b0;
                end else begin
                    // Normal operation when no stall is required
                    state <= `State_Stall_IDLE;

                    // No stalls needed
                    Stall1_CAL <= 1'b0;
                    Stall2_CAL <= 1'b0;
                    Stall3_CAL <= 1'b0;

                    // Increment PC normally
                    load_PC_CAL <= 1'b1;

                    // Load instructions normally
                    load_ir_CAL <= 1'b1;

                    WRITE_MEM <= `MEM_NONE;
                    REGWRITELDR <= 1'b0;
                end
            end

            `State_Stall_ACTIVE: begin
                // Release the stall after one cycle
                state <= `State_Stall_IDLE;

                // Remove stalls
                Stall1_CAL <= 1'b0;
                Stall2_CAL <= 1'b0;
                Stall3_CAL <= 1'b0;

                // Resume normal PC increment
                load_PC_CAL <= 1'b1;

                // Allow instruction register to load new instruction
                load_ir_CAL <= 1'b1;

                // Disable memory write and REGWRITELDR after stall
                WRITE_MEM <= `MEM_NONE;
                REGWRITELDR <= 1'b1;
            end

            default: begin
                state <= `State_Stall_IDLE;

                // Default values
                Stall1_CAL <= 1'b0;
                Stall2_CAL <= 1'b0;
                Stall3_CAL <= 1'b0;
                load_PC_CAL <= 1'b1;
                WRITE_MEM <= `MEM_NONE;
                REGWRITELDR <= 1'b0;
                load_ir_CAL <= 1'b1;
            end
        endcase
    end
endmodule



module PIPELINE_EXFI(ALUout, Zflags, PC, loadsP, opcodeP, opcodePP, loadsPP, ALUoutP, ZflagsP, PCP, clk);

    input clk; 
    input [15:0] ALUout; 
    input [2:0] Zflags, opcodeP; 
    input [8:0] PC; 
    input loadsP;

    output reg [15:0] ALUoutP; 
    output reg [2:0] ZflagsP, opcodePP;
    output reg [8:0] PCP; 
    output reg loadsPP;

    always_ff @(posedge clk) begin 
        ALUoutP <= ALUout;
        ZflagsP <= Zflags; 
        PCP <= PC;
        loadsPP <= loadsP; 
        opcodePP <= opcodeP;
    end

endmodule : PIPELINE_EXFI
module PIPELINE_IDEX(Rd,  Rn,  Rm,  opcode,  op,  cond,  ALUop,  shift,  sximm8,  sximm5,  data_in, PC, REGWRITE, VSEL2, SelectAIN, SelectBIn, loads,
                     RdP, RnP, RmP, opcodeP, opP, condP, ALUopP, shiftP, sximm8P, sximm5P, data_inP, PCP, REGWRITEP, VSEL2P, SelectAINP, SelectBInP, loadsP, 
                     Flush3, Stall3, clk); 

    input Flush3, Stall3, clk;
    input [2:0] Rd, Rn, Rm, opcode, cond;
    input [1:0] op, shift, ALUop; 
    input [15:0] sximm5, sximm8, data_in;
    input [8:0] PC;
    input REGWRITE, SelectAIN, SelectBIn, loads;
    input [1:0] VSEL2; 

    output [2:0] RdP, RnP, RmP, opcodeP, condP;
    output [1:0] opP, shiftP, ALUopP; 
    output [15:0] sximm5P, sximm8P, data_inP;
    output [8:0] PCP; 
    output REGWRITEP, SelectAINP, SelectBInP, loadsP;
    output [1:0] VSEL2P; 

    reg [2:0] RdP_CAL, RnP_CAL, RmP_CAL, opcodeP_CAL, condP_CAL;
    reg [1:0] opP_CAL, shiftP_CAL, ALUop_CAL; 
    reg [15:0] sximm5P_CAL, sximm8P_CAL, data_inP_CAL;
    reg [8:0] PCP_CAL; 
    reg REGWRITE_CALP, SelectAIN_CALP, SelectBIn_CALP, loads_CALP;
    reg [1:0] VSEL2_CALP; 

    assign RdP = RdP_CAL;
    assign RnP = RnP_CAL;
    assign RmP = RmP_CAL;
    assign opcodeP = opcodeP_CAL;
    assign condP = condP_CAL;
    assign opP = opP_CAL;
    assign shiftP = shiftP_CAL;
    assign sximm5P = sximm5P_CAL;
    assign sximm8P = sximm8P_CAL;
    assign data_inP = data_inP_CAL;
    assign ALUopP = ALUop_CAL;
    assign REGWRITEP = REGWRITE_CALP;
    assign SelectAINP = SelectAIN_CALP;
    assign SelectBInP = SelectBIn_CALP;
    assign loadsP = loads_CALP;
    assign VSEL2P = VSEL2_CALP;

    assign PCP = PCP_CAL;

    always_ff @(posedge clk) begin 
        if(Flush3) begin 
            RdP_CAL <= 3'd0;
            RnP_CAL <= 3'd0;
            RmP_CAL <= 3'd0;
            opcodeP_CAL <= 3'd0; 
            condP_CAL <= 3'd0;
            opP_CAL <= 2'd0;
            shiftP_CAL <= 2'd0;
            sximm5P_CAL <= 16'd0;
            sximm8P_CAL <= 16'd0; 
            data_inP_CAL <= 16'd0;
            PCP_CAL <= 9'd0;
            ALUop_CAL <= 2'd0;
            REGWRITE_CALP <= 1'b0;
            SelectAIN_CALP <= 1'b0;
            SelectBIn_CALP <= 1'b0;
            loads_CALP <= 1'b0;
            VSEL2_CALP <= 2'd0;
        end else if(Stall3) begin 
            RdP_CAL <= RdP_CAL;
            RnP_CAL <= RnP_CAL;
            RmP_CAL <= RmP_CAL;
            opcodeP_CAL <= opcodeP_CAL; 
            condP_CAL <= condP_CAL;
            opP_CAL <= opP_CAL;
            shiftP_CAL <= shiftP_CAL;
            sximm5P_CAL <= sximm5P_CAL;
            sximm8P_CAL <= sximm8P_CAL; 
            data_inP_CAL <= data_inP_CAL;
            PCP_CAL <= PCP_CAL; 
            ALUop_CAL <= ALUop_CAL;
            REGWRITE_CALP <= REGWRITE_CALP;
            SelectAIN_CALP <= SelectAIN_CALP;
            SelectBIn_CALP <= SelectBIn_CALP;
            loads_CALP <= loads_CALP;
            VSEL2_CALP <= VSEL2_CALP;
        end else begin 
            RdP_CAL <= Rd;
            RnP_CAL <= Rn;
            RmP_CAL <= Rm;
            opcodeP_CAL <= opcode; 
            condP_CAL <= cond;
            opP_CAL <= op;
            shiftP_CAL <= shift;
            sximm5P_CAL <= sximm5;
            sximm8P_CAL <= sximm8; 
            data_inP_CAL <= data_in;
            PCP_CAL <= PC; 
            ALUop_CAL <= ALUop;
            REGWRITE_CALP <= REGWRITE;
            SelectAIN_CALP <= SelectAIN;
            SelectBIn_CALP <= SelectBIn;
            loads_CALP <= loads;
            VSEL2_CALP <= VSEL2;
        end
    end

endmodule : PIPELINE_IDEX


module PIPELINE_IFID(PCG, PCOUT, INSTRUCTION_READ, INSTRUCTION_DECODE, Flush2, Stall2, clk); 
    input Flush2, Stall2, clk; 
    input [8:0] PCG; 
    output [8:0] PCOUT; 
    input [15:0] INSTRUCTION_READ;
    output [15:0] INSTRUCTION_DECODE; 

    reg [8:0] PCCAL;
    reg [15:0] INSTR_CAL; 

    assign INSTRUCTION_DECODE = INSTR_CAL;
    assign PCOUT = PCCAL; 

    always_ff @(posedge clk) begin
        if (Flush2) begin
            INSTR_CAL <= 16'd0;
            PCCAL <= 9'd0; 
        end else if (Stall2) begin
            INSTR_CAL <= INSTR_CAL;
            PCCAL <= PCCAL;
        end else begin
            INSTR_CAL <= INSTRUCTION_READ;
            PCCAL <= PCG;
        end
    end

endmodule : PIPELINE_IFID


module PIPELINE_PCIF(PC, address, Flush1, Stall1, clk);

    input Flush1, Stall1, clk;
    input [8:0] PC; 
    output [8:0] address; 
    reg [8:0] address_inCal;

    assign address = address_inCal; 

    always_ff @(posedge clk) begin
        if (Flush1) begin
            address_inCal <= PC; 
        end else if (Stall1) begin
            address_inCal <= address_inCal; 
        end else begin
            address_inCal <= PC;
        end
    end

endmodule : PIPELINE_PCIF

module InstaDecoder(instructionReg, opcode, op, ALUop, sximm5, sximm8, shift, Rd, Rn, Rm, cond);
    input [15:0] instructionReg;
    output [15:0] sximm5, sximm8;
    output wire [2:0] Rd, Rn, Rm, opcode, cond;
    output [1:0] op, ALUop, shift;

    assign Rd = ({instructionReg[15:13], instructionReg[12:11]} == 5'b11010 || {instructionReg[15:13], instructionReg[12:11]} == 5'b01011) ? instructionReg[10:8] : instructionReg[7:5];
    assign Rn = instructionReg[10:8];
    assign Rm = instructionReg[2:0];
    assign cond = instructionReg[10:8];
    assign opcode = instructionReg[15:13];
    assign op = instructionReg[12:11];
    assign ALUop = instructionReg[12:11];
    assign shift = {instructionReg[15:13]} == 3'b100 ? 2'b00 : instructionReg[4:3];
    assign sximm5 = {{11{instructionReg[4]}}, instructionReg[4:0]};
    assign sximm8 = {{8{instructionReg[7]}}, instructionReg[7:0]};

endmodule : InstaDecoder
module ProgramCounter(PCout, clk, reset, enablePC, load_PC, imJump, imJumpFlag);
    input clk, reset, enablePC, imJumpFlag, load_PC;
    input [8:0] imJump;
    output wire [8:0] PCout;

    reg [8:0] PCoutInCal; 

    assign PCout = PCoutInCal;

    always_ff @(posedge clk) begin 
        if(reset) begin 
            PCoutInCal <= 9'd0;
        end else begin 
            if(imJumpFlag) begin 
                PCoutInCal <= imJump;
            end else if (load_PC) begin 
                PCoutInCal <= PCoutInCal + 9'd1;
            end else begin
                PCoutInCal <= PCoutInCal;
            end
        end
    end

endmodule : ProgramCounter

module PIPELINEREGISTER(indata, outdata, clk);
    parameter WIDTH = 16;
    input [WIDTH-1:0] indata;
    output [WIDTH-1:0] outdata;
    input clk;

    reg [WIDTH-1:0] outdataInCal;

    assign outdata = outdataInCal;

    always_ff @(posedge clk) begin
       outdataInCal <= indata;
    end

endmodule : PIPELINEREGISTER