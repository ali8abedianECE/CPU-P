module EXME(clk, reset, ALUResultE, WriteDataE, RegWriteE, MemWriteE, Z, ALUResultM, WriteDataM, RegWriteM, MemWriteM, N, V, Z);
    input clk;
    input reset;
    input [15:0] ALUResultE;
    input [15:0] WriteDataE;
    input RegWriteE;
    input MemWriteE;
    input [2:0] Z;
    output wire [15:0] ALUResultM;
    output wire [15:0] WriteDataM;
    output wire RegWriteM;
    output wire MemWriteM;
    //Z[0] = Z, Z[1] = N, Z[2] = V
    output wire N, V, Z;

    PIPELINEREGISTER #(16) ALUResultReg(.indata(ALUResultE), .outdata(ALUResultM), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(16) WriteDataReg(.indata(WriteDataE), .outdata(WriteDataM), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) RegWriteReg(.indata(RegWriteE), .outdata(RegWriteM), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) MemWriteReg(.indata(MemWriteE), .outdata(MemWriteM), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) ZReg(.indata(Z[0]), .outdata(Z), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) NReg(.indata(Z[1]), .outdata(N), .clk(clk), .reset(reset));
    PIPELINEREGISTER #(1) VReg(.indata(Z[2]), .outdata(V), .clk(clk), .reset(reset));

endmodule : EXME