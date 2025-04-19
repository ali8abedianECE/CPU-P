module MUX3a(out, in0, in1, in2, select);
  parameter k = 1;
  input [k-1:0] in0, in1, in2;
  output reg [k-1:0] out;
  input [1:0] select; 
  always_comb begin
    case(select)
      0: out = in0;
      1: out = in1;
      2: out = in2;
      default: out = {k{1'bx}};
    endcase
  end

endmodule : MUX3a