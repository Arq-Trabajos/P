`include "alu.v"
module alu_tb;

    reg [2:0] ALUControl; 
    reg [31:0] A; 
    reg [31:0] B;  // Declaration ofinputs and ALUControl signal, A & B support 32 bits
    wire [3:0] ALUFlags;
    wire [31:0] Result; 
    wire [31:0] ResultExtra;

    alu ALUtest(ALUControl, A, B, ALUFlags, Result, ResultExtra);

    initial begin

        ALUControl = 3'b110;

        A = -2;
        B = 3;

        #1;
        $display("%x %x %x %x %x %x",ALUControl, A, B, ALUFlags, Result, ResultExtra);


    end

endmodule