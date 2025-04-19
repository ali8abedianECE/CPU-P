
module datapath_tb();
    reg clk, reset;
    reg enablePC, imJumpFlag, Flush1, Flush2, Flush3;
    reg [15:0] imJump;
    wire Zout, Nout, Vout;
    wire [15:0] OUT;
    wire HALT;
    wire [8:0] PCFinal;

    // Instantiate datapath
    datapath DUT(
        .clk(clk),
        .reset(reset),
        .enablePC(enablePC),
        .Zout(Zout),
        .Nout(Nout),
        .Vout(Vout),
        .OUT(OUT),
        .HALT(HALT), 
        .PCFinal(PCFinal)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // // Initialize instruction memory
    // initial begin
    //     // Load instructions into memory
    //     DUT.READ_ONLY_MEM.mem[0] = 16'b1101_0_001_00000101;  // MOV R1, #5
    //     DUT.READ_ONLY_MEM.mem[1] = 16'b1101_0_010_00000101;  // MOV R2, #5
    //     DUT.READ_ONLY_MEM.mem[2] = 16'b1010_1_001_000_00_010;// CMP R1, R2
    //     DUT.READ_ONLY_MEM.mem[3] = 16'b0010_0001_00000010;   // BEQ +2 (equal)
    //     DUT.READ_ONLY_MEM.mem[4] = 16'b1101_0_011_00001010;  // MOV R3, #10 (skipped)
    //     DUT.READ_ONLY_MEM.mem[5] = 16'b1101_0_011_00001111;  // MOV R3, #15 (equal:)

    //     DUT.READ_ONLY_MEM.mem[6] = 16'b1101_0_001_00010100;  // MOV R1, #20
    //     DUT.READ_ONLY_MEM.mem[7] = 16'b1010_1_001_000_00_010;// CMP R1, R2
    //     DUT.READ_ONLY_MEM.mem[8] = 16'b0010_0010_00000010;   // BNE +2 (noteq)
    //     DUT.READ_ONLY_MEM.mem[9] = 16'b1101_0_100_00011001;  // MOV R4, #25 (skipped)
    //     DUT.READ_ONLY_MEM.mem[10] = 16'b1101_0_100_00011110; // MOV R4, #30 (noteq:)

    //     DUT.READ_ONLY_MEM.mem[11] = 16'b1101_0_001_00000001; // MOV R1, #1
    //     DUT.READ_ONLY_MEM.mem[12] = 16'b1101_0_010_00001010; // MOV R2, #10
    //     DUT.READ_ONLY_MEM.mem[13] = 16'b1010_1_001_000_00_010;// CMP R1, R2
    //     DUT.READ_ONLY_MEM.mem[14] = 16'b0010_0011_00000010;  // BLT +2 (less)
    //     DUT.READ_ONLY_MEM.mem[15] = 16'b1101_0_101_00100011; // MOV R5, #35 (skipped)
    //     DUT.READ_ONLY_MEM.mem[16] = 16'b1101_0_101_00101000; // MOV R5, #40 (less:)

    //     DUT.READ_ONLY_MEM.mem[17] = 16'b0010_0000_00000010;  // B +2 (end)
    //     DUT.READ_ONLY_MEM.mem[18] = 16'b1101_0_110_00101101; // MOV R6, #45 (skipped)
    //     DUT.READ_ONLY_MEM.mem[19] = 16'b1101_0_110_00110010; // MOV R6, #50 (end:)
    // end

    // Test stimulus
    initial begin
        // Initialize signals
        reset = 1;
        enablePC = 0;
        imJumpFlag = 0;
        Flush1 = 0;
        Flush2 = 0;
        Flush3 = 0;
        imJump = 16'b0;

        // Reset sequence
        @(posedge clk);
        reset = 0;
        enablePC = 1;

        // Let it run
        repeat(500) @(posedge clk);
        
        $stop;
    end
    

    // Monitor changes
    initial begin
        $monitor("Time=%0t PC=%h reset=%b enablePC=%b HALT=%b R0=%h R1=%h R2=%h R3=%h R4=%h R5=%h R6=%h R7=%h mem[0X14]=%h, Zout=%b Nout=%b Vout=%b",
                 $time, 
                 PCFinal,
                 reset, 
                 enablePC, 
                 HALT,
                 DUT.REGFILE.R0, 
                 DUT.REGFILE.R1, 
                 DUT.REGFILE.R2, 
                 DUT.REGFILE.R3, 
                 DUT.REGFILE.R4,
                 DUT.REGFILE.R5,
                 DUT.REGFILE.R6,
                 DUT.REGFILE.R7,
                 DUT.MEM.mem[8'h14],
                 Zout, Nout, Vout);
    end
endmodule