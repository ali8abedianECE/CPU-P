module IFID(clk, reset, InstrF, PCin, InstrD, PCout);
    input clk, reset;
    input [15:0] InstrF;
    input [8:0] PCin;
    output reg [15:0] InstrD;
    output reg [8:0] PCout;

    PIPELINEREGISTER #(16) InstrReg(.indata(InstrF), .outdata(InstrD), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(9) PCReg(.indata(PCin), .outdata(PCout), .clk(clk), .reset(reset));

endmodule : IFID