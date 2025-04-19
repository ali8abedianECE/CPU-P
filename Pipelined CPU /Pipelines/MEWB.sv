module MEWB(clk, reset, ReadDataM, ALUResultM, RegWriteM, ReadDataW, ALUResultW, RegWriteW);
    input clk;
    input reset;
    input [15:0] ReadDataM;
    input [15:0] ALUResultM;
    input RegWriteM;
    output wire [15:0] ReadDataW;
    output wire [15:0] ALUResultW;
    output wire RegWriteW;

    PIPELINEREGISTER #(16) ReadDataReg(.indata(ReadDataM), .outdata(ReadDataW), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) ALUResultReg(.indata(ALUResultM), .outdata(ALUResultW), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) RegWriteReg(.indata(RegWriteM), .outdata(RegWriteW), .clk(clk), .reset(reset));

endmodule : MEWB
