module shifter(in, shift, sout);
    input [15:0] in;
    input [1:0] shift;
    output reg [15:0] sout;
    reg tempSto; 
    always_comb begin
        tempSto = 1'b0; 
        case(shift) 
            2'b00: sout = in; 
            2'b01: sout = in << 1; 
            2'b10: sout = in >> 1; 
            2'b11: begin 
                tempSto = in[15];
                sout = in >> 1; 
                sout[15] = tempSto; 
            end
            default: sout = {16{1'bx}};
        endcase
    end
endmodule : shifter