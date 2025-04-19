module MUX2a(out, input0, input1, select);
  parameter k = 1;
  input [k-1:0] input0, input1;
  output reg [k-1:0] out;
  input select; 
  always_comb begin
    case(select)
      0: out = input0;
      1: out = input1;
      default: out = {k{1'bx}};
    endcase
  end
endmodule : MUX2a