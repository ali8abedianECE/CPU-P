//
// 1. "Example 12-11: Verilog Single Clock Simple Dual-Port Synchronous RAM 
//     with Old Data Read-During-Write Behavior" 
// 2. "Example 12-29: Verilog HDL RAM Initialized with the readmemb Command"

module RAM2(clk, read_address, write_address, write, din, dout);
parameter data_width = 16; 
parameter addr_width = 9;
parameter filename = "data.txt";

input clk;
input [addr_width-1:0] read_address, write_address;
input write;
input [data_width-1:0] din;
output [data_width-1:0] dout;
reg [data_width-1:0] dout;

reg [data_width-1:0] mem [2**addr_width-1:0];

initial $readmemb(filename, mem);

always @ (posedge clk) begin
  if (write)
    mem[write_address] <= din;
  dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                             // (this is due to Verilog non-blocking assignment "<=")
end 
endmodule : RAM2
