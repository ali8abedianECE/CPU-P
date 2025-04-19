module cpu(clk, reset, OUT, N, V, Z, HALT, MEM_CMD, dinRam, ReadMEM);
    input clk, reset;
    input [15:0] ReadMEM; 
    output wire HALT;
    reg [8:0] PC; 
    wire [8:0] PCC, PCCCC;
    output wire [1:0] MEM_CMD;
    
    output wire [15:0] dinRam; 
    output wire  [15:0] OUT;
    output wire N, V, Z; 
    reg resetREM; 

    always_ff @(posedge clk) begin 
        resetREM <= reset;
    end

    always_comb begin
        if(reset) begin 
            PC = 9'd0; //accounts for offset of the pipeline timings 
        end else if(resetREM) begin 
            PC = 9'd0;
        end else begin 
            PC = PCCCC; //accounts for offset of the pipeline timings
        end
    end

    datapath DP(.clk(clk), .reset(reset), .enablePC(~reset), .Zout(Z), .Nout(N), .Vout(V), .OUT(OUT), .HALT(HALT), .PCFinal(PCC), .PCCCC(PCCCC), .MEM_CMD(MEM_CMD), .dinRam(dinRam), .ReadMEM(ReadMEM));

endmodule : cpu