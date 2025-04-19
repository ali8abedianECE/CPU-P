module IDEX(clk, reset, RnE, RmE, InstrE, ImmExtE8, ImmExtE5, ALUopE, shiftE, RnNumE, RmNumE, RdNumE,writeE, RnNum, RmNum, RdNum, Rn, Rm, ImmExt5, ImmExt8, ALUop, write, shift);
    input clk;
    input reset;
    input [15:0] Rn, Rm, ImmExt5, ImmExt8;
    input [2:0] RnNum, RmNum, RdNum;
    input write; 
    input [1:0] shift; 
    input [1:0] ALUop;

    output wire [15:0] RnE, RmE, InstrE, ImmExtE8, ImmExtE5;
    output wire [2:0] RnNumE, RmNumE, RdNumE;
    output wire [1:0] ALUopE, shiftE;
    output wire writeE;

    PIPELINEREGISTER #(16) RnReg(.indata(Rn), .outdata(RnE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) RmReg(.indata(Rm), .outdata(RmE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) ImmExt5Reg(.indata(ImmExt5), .outdata(ImmExtE5), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) ImmExt8Reg(.indata(ImmExt8), .outdata(ImmExtE8), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) InstrReg(.indata(InstrE), .outdata(InstrE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(3) RnNumReg(.indata(RnNum), .outdata(RnNumE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(3) RmNumReg(.indata(RmNum), .outdata(RmNumE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(3) RdNumReg(.indata(RdNum), .outdata(RdNumE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(2) ALUopReg(.indata(ALUop), .outdata(ALUopE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) RegWriteReg(.indata(write), .outdata(writeE), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(2) shiftReg(.indata(shift), .outdata(shiftE), .clk(clk), .reset(reset));
    
endmodule : IDEX