
module lab7_bonousTB(); 
    reg [3:0] KEY;
    reg [9:0] SW;
    wire [9:0] LEDR; 
    wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
    reg err;
    reg CLOCK_50;
    wire [15:0] memlook, reg1, reg2, reg3, reg4, reg5;
    wire [8:0] PC;
    wire [6:0] State;

    lab7bonus_top DUT(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);

    initial forever begin
        CLOCK_50 = 0; #5;
        CLOCK_50 = 1; #5;
    end

    wire break_ = (LEDR[8]);
    initial begin
        err = 0;
        KEY[1] = 1'b0; // reset asserted
        #10; // wait until next falling edge of clock
        KEY[1] = 1'b1; // reset de-asserted, PC still undefined if as in Figure 4
        
        while (~break_) begin
        // Change the following line to wait until your CPU starts to you fetch
        // the next instruction (e.g., IF1 state from Lab 7 or equivalent in
        // your design).  DUT.CPU.FSM is not required for by the autograder
        // for Lab 8. 
        @(posedge break_);  

        @(negedge CLOCK_50); // show advance to negative edge of clock
        $display("PC = %h", DUT.CPU.PC); 
        end
        $display("Time=%0t PC=%h HALT=%b R0=%h R1=%h R2=%h R3=%h R4=%h R5=%h R6=%h R7=%h mem[0X14]=%h, Zout=%b Nout=%b Vout=%b",
            $time, 
            DUT.CPU.PC,
            DUT.HALT,
            DUT.CPU.DP.REGFILE.R0, 
            DUT.CPU.DP.REGFILE.R1, 
            DUT.CPU.DP.REGFILE.R2, 
            DUT.CPU.DP.REGFILE.R3, 
            DUT.CPU.DP.REGFILE.R4,
            DUT.CPU.DP.REGFILE.R5,
            DUT.CPU.DP.REGFILE.R6,
            DUT.CPU.DP.REGFILE.R7,
            DUT.MEM.mem[8'h14],
            DUT.Z, DUT.N, DUT.V);
        $stop;
    end

endmodule