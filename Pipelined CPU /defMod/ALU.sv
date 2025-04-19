`define add 2'b00
`define subtract 2'b01
`define and 2'b10
`define not 2'b11

module ALU(Ain, Bin, ALUop, out, Z);
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output reg [15:0] out;
    //Z[0] = Z, Z[1] = N, Z[2] = V
    //Z = 1 if the result is zero
    //N = 1 if the result is negative
    //V = 1 if the result is overflow
    output reg [2:0] Z;

    assign Z[0] = out === 16'd0 ? 1 : 0;
    assign Z[1] = out[15];

    //TODO change the block such that it handels the condition of out == 0; think
    // it works now but not sure. READ THE LAB SHEET
    always_comb begin 
        Z[2] = 1'b0; 
        case(ALUop) 
            `add: begin 
                out = Ain + Bin;
                if((Ain[15] & Bin[15] & ~out[15]) | ((~Ain[15] & ~Bin[15] & out[15]))) begin 
                    Z[2] = 1'b1; 
                end 
            end
            `subtract: begin 
                out = Ain - Bin;
                if((Ain[15] & ~Bin[15] & ~out[15]) | ((~Ain[15] & Bin[15] & out[15]))) begin 
                    Z[2] = 1'b1; 
                end 
            end
            `and: begin 
                out = Ain & Bin; 
            end
            `not: begin 
                out = ~Bin;
            end
            default: out = {16{1'bx}};
        endcase
    end

endmodule : ALU