`define MEM_NONE 2'b00
`define MEM_READ 2'b01
`define MEM_WRITE 2'b10
module lab7bonus_top(KEY, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, CLOCK_50);
    input [3:0] KEY;
    input [9:0] SW;
    input CLOCK_50;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    wire [15:0] read_data;
    wire [15:0] write_data;
    wire N, V, Z, HALT;
    wire [1:0] mem_cmd;
    wire addr_sel;
    wire load_pc, load_ir;
    wire [8:0] mem_addr;
    wire [15:0] read_data_from_ram;
    wire [15:0] ReadMEM; 
    wire [1:0] MEM_CMD;
    wire [15:0] dinRam; 
    wire [15:0] OUT;
    wire isHalt;
    wire [7:0] LEDS;

    cpu CPU(.clk(CLOCK_50), .reset(~KEY[1]), .OUT(OUT), .N(N), .V(V), .Z(Z), .HALT(HALT), .MEM_CMD(MEM_CMD), .dinRam(dinRam), .ReadMEM(ReadMEM));
    RAM #(16, 8) MEM(.clk(CLOCK_50), .read_address(OUT[7:0]), .write_address(OUT[7:0]), .write((MEM_CMD == `MEM_WRITE) & OUT[8] == 1'b0), .din(dinRam), .dout(read_data_from_ram));

    assign ReadMEM =  read_data_from_ram;
    //Reg #(1) LEDRU(.in(HALT), .load(1'b1), .out(LEDR[8]), .clk(CLOCK_50));
    assign LEDR[8] = HALT == 1'b1 ? 1'b1 : 1'b0;
    assign LEDR[9] = 1'b0;
    assign LEDR[7:0] = LEDS;

    LEDS LED(.LEDR(LEDS), .mem_cmd(MEM_CMD), .mem_addr(OUT[8:0]), .write_data(OUT), .clk(CLOCK_50));
    SliderSwitch SW1(.SW(SW), .mem_cmd(MEM_CMD), .mem_addr(OUT[8:0]), .read_data(read_data));

    sseg H0(OUT[3:0],   HEX0);
    sseg H1(OUT[7:4],   HEX1);
    sseg H2(OUT[11:8],  HEX2);
    sseg H3(OUT[15:12], HEX3);
    assign HEX4 = 7'b1111111;
    assign {HEX5[2:1],HEX5[5:4]} = 4'b1111; // disabled
    assign HEX5[0] = ~Z;
    assign HEX5[6] = ~N;
    assign HEX5[3] = ~V;

endmodule : lab7bonus_top

module SliderSwitch(SW, mem_cmd, mem_addr, read_data); 
    input [9:0] SW;
    input [1:0] mem_cmd;
    input [8:0] mem_addr;
    output [15:0] read_data;

    assign read_data = (mem_cmd == `MEM_READ & mem_addr == 9'h140) ? {8'h00, SW[7:0]}: 16'bz;

endmodule : SliderSwitch

module LEDS(LEDR, mem_cmd, mem_addr, write_data, clk);
    input [15:0] write_data;
    input [1:0] mem_cmd;
    input [8:0] mem_addr;
    input clk;
    output [7:0] LEDR;

    Reg #(8) led_reg(.in({write_data[7:0]}), .load((mem_cmd == `MEM_WRITE) & (mem_addr == 9'h100)), .clk(clk), .out(LEDR));

endmodule : LEDS

`define SevenSeg0 7'b1000000
`define SevenSeg1 7'b1111001
`define SevenSeg2 7'b0100100
`define SevenSeg3 7'b0110000
`define SevenSeg4 7'b0011001
`define SevenSeg5 7'b0010010
`define SevenSeg6 7'b0000010
`define SevenSeg7 7'b1111000
`define SevenSeg8 7'b0000000
`define SevenSeg9 7'b0010000

`define Reset 7'b1111111
`define CharA 7'b0001000
`define CharB 7'b0000011
`define CharC 7'b1000110
`define CharD 7'b0100001
`define CharE 7'b0000110
`define CharF 7'b0001110


module sseg(in, segs);
  input [3:0] in;
  output reg [6:0] segs;

  always_comb begin 

    case(in) 
      4'd0: segs = `SevenSeg0;
      4'd1: segs = `SevenSeg1;
      4'd2: segs = `SevenSeg2;
      4'd3: segs = `SevenSeg3;
      4'd4: segs = `SevenSeg4;
      4'd5: segs = `SevenSeg5;
      4'd6: segs = `SevenSeg6;
      4'd7: segs = `SevenSeg7;
      4'd8: segs = `SevenSeg8;
      4'd9: segs = `SevenSeg9;
      4'd10: segs = `CharA;
      4'd11: segs = `CharB;
      4'd12: segs = `CharC;
      4'd13: segs = `CharD;
      4'd14: segs = `CharE;
      4'd15: segs = `CharF;
      default: segs = {7{1'bx}};
    endcase

  end

endmodule : sseg