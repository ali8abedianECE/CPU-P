    module MUX8a(toRead, reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, data_out);
    parameter k = 1; 
    input [k-1:0] reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7; 
    input [2:0] toRead;
    output reg [k-1:0] data_out; 
    always_comb begin 
        case(toRead)
            0: data_out = reg0;
            1: data_out = reg1;
            2: data_out = reg2;
            3: data_out = reg3;
            4: data_out = reg4;
            5: data_out = reg5;
            6: data_out = reg6;
            7: data_out = reg7; 
            default: data_out = {k{1'bx}};
        endcase
    end
endmodule : MUX8a