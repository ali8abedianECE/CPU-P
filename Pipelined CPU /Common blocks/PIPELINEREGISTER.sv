module PIPELINEREGISTER(indata, outdata, clk, reset, stall);
    parameter WIDTH = 16;
    input [WIDTH-1:0] indata;
    output [WIDTH-1:0] outdata;
    input clk, reset, stall;

    reg [WIDTH-1:0] outdataInCal;

    assign outdata = ~stall ? outdataIncal : {WIDTH{1'bz}};

    always_ff @(posedge clk) begin
        if (reset) begin
            outdataInCal <= 16'b0;
        end else begin
            outdataInCal <= indata;
        end
    end
    
endmodule : PIPELINEREGISTER