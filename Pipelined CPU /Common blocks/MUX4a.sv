module MUX4a(out, in0, in1, in2, in3, select);
  parameter k = 1;
  input [k-1:0] in0, in1, in2, in3;
  output reg [k-1:0] out;
  input [1:0] select; 
  always_comb begin
    case(select)
      0: out = in0;
      1: out = in1;
      2: out = in2;
      3: out = in3;
      default: out = {k{1'bx}};
    endcase
  end
endmodule : MUX4a