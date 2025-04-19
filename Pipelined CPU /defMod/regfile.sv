module regfile(data_in, writenum, write, RnNUM, RmNUM, RdNUM, RdNUMREAD, Rn, Rm, Rd, RdREAD, clk);
    input [15:0] data_in;
    input [2:0] writenum, RnNUM, RmNUM, RdNUM, RdNUMREAD;
    input write, clk;
    output wire [15:0] Rn, Rm, Rd, RdREAD;

    wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;
    wire [7:0] decoded_preand;
    wire [7:0] decoded_postand;

    MUX8a #(16) RMMUX(
        .toRead(RmNUM),
        .reg0(R0), .reg1(R1),
        .reg2(R2), .reg3(R3),
        .reg4(R4), .reg5(R5),
        .reg6(R6), .reg7(R7),
        .data_out(Rm)
    );

    MUX8a #(16) RNMUX(
        .toRead(RnNUM),
        .reg0(R0), .reg1(R1),
        .reg2(R2), .reg3(R3),
        .reg4(R4), .reg5(R5),
        .reg6(R6), .reg7(R7),
        .data_out(Rn)
    );

    MUX8a #(16) RDMUX(
        .toRead(RdNUM),
        .reg0(R0), .reg1(R1),
        .reg2(R2), .reg3(R3),
        .reg4(R4), .reg5(R5),
        .reg6(R6), .reg7(R7),
        .data_out(Rd)
    );

    MUX8a #(16) RD2(
        .toRead(RdNUMREAD),
        .reg0(R0), .reg1(R1),
        .reg2(R2), .reg3(R3),
        .reg4(R4), .reg5(R5),
        .reg6(R6), .reg7(R7),
        .data_out(RdREAD)
    );

    dec #(3, 8) decoder38(.in(writenum), .out(decoded_preand));
    assign decoded_postand = decoded_preand & {8{write}};

    Reg register0(.in(data_in), .load(decoded_postand[0]), .clk(clk), .out(R0));
    Reg register1(.in(data_in), .load(decoded_postand[1]), .clk(clk), .out(R1));
    Reg register2(.in(data_in), .load(decoded_postand[2]), .clk(clk), .out(R2));
    Reg register3(.in(data_in), .load(decoded_postand[3]), .clk(clk), .out(R3));
    Reg register4(.in(data_in), .load(decoded_postand[4]), .clk(clk), .out(R4));
    Reg register5(.in(data_in), .load(decoded_postand[5]), .clk(clk), .out(R5));
    Reg register6(.in(data_in), .load(decoded_postand[6]), .clk(clk), .out(R6));
    Reg register7(.in(data_in), .load(decoded_postand[7]), .clk(clk), .out(R7));

endmodule : regfile
