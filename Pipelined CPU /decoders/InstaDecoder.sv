module InstaDecoder(instructionReg, opcode, op, ALUop, sximm5, sximm8, shift, Rd, Rn, Rm, cond);
    input [15:0] instructionReg;
    output [15:0] sximm5, sximm8;
    output wire [2:0] Rd, Rn, Rm, opcode, cond;
    output [1:0] op, ALUop, shift;
   
    assign Rd = {opcode, op} == 5'b11010 ? instructionReg[10:8] : instructionReg[7:5];
    assign Rn = instructionReg[10:8];
    assign Rm = instructionReg[2:0];
    assign cond = instructionReg[10:8];
    assign opcode = instructionReg[15:13];
    assign op = instructionReg[12:11];
    assign ALUop = instructionReg[12:11];
    assign shift = instructionReg[4:3];
    assign sximm5 = {{11{instructionReg[4]}}, instructionReg[4:0]};
    assign sximm8 = {{8{instructionReg[7]}}, instructionReg[7:0]};

endmodule : InstaDecoder