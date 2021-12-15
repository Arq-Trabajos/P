`include "alu.v"
`timescale 1ns/1ns
module alu_tb;

    reg [2:0] ALUControl; 
    reg [31:0] A; 
    reg [31:0] B;  // Declaration ofinputs and ALUControl signal, A & B support 32 bits
    wire [3:0] ALUFlags;
    wire [31:0] Result; 
    wire [31:0] ResultExtra;

    alu ALUtest(ALUControl, A, B, ALUFlags, Result, ResultExtra);

    initial begin
        ALUControl = 3'b100;
        A = -2;
        B = 3;
        #1;
        $display("MUL(%b): %x x %x = (lower 32) %x (possible overflow) %x; Flags: %b",ALUControl, A, B, Result, ResultExtra, ALUFlags);

        ALUControl = 3'b110;
        A = -5;
        B = 10;
        #1;
        $display("SMULL(%b): %x x %x = (lower 32) %x (higher 32) %x; Flags: %b",ALUControl, A, B, Result, ResultExtra, ALUFlags);


        ALUControl = 3'b111;
        A = 10;
        B = 45;
        #1;
        $display("UMULL(%b): %x x %x = (lower 32) %x (higher 32) %x; Flags: %b",ALUControl, A, B, Result, ResultExtra, ALUFlags);
		$dumpfile("alutb.vcd");
		$dumpvars;
        $finish;
	    
 
    end
endmodule