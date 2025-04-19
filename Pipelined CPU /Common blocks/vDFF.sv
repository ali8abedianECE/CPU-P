module vDFF(in, clk, out);
  parameter k = 1;
  input [k-1:0] in;
  output reg [k-1:0] out;
  input clk;
  always_ff @(posedge clk) begin 
    out <= in;
  end
endmodule : vDFF