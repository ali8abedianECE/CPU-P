module Reg(in, load, clk, out);
  parameter k = 16;
  input [k-1:0] in;
  output wire [k-1:0] out;
  input clk, load;
  wire [k-1:0] outOfMux; 
  vDFF #(k) dff0(.in(outOfMux), .clk(clk), .out(out));
  MUX2a #(k) mux0(.out(outOfMux), .input0(out), .input1(in), .select(load));
endmodule : Reg