module alu (
	input [2:0] ALUControl, input [31:0] A, input [31:0] B,  // Declaration ofinputs and ALUControl signal, A & B support 32 bits
	output [3:0] ALUFlags, output reg [31:0] Result, output reg [31:0] ResultExtra // Declaration of module outputs 
);
	wire [31:0] mux2_1; // Wire to hold the output of the mux2:1 
	wire [32:0] Sum; // Wire that will hold the sum of A and B if it is required
	wire Overflow, Carry,Negative, Zero; // Wires that hold each ALU Flag
    
	assign mux2_1 = (ALUControl[0]==0)? B: ~B; // MUX 2:1 simplification using the conditional operator
	assign Sum = A+mux2_1+ALUControl[0]; // Definition of the ADDER module

	// MUX 4:1 simplification using the CASE keyword & syntax
	always @(*) begin
		casex (ALUControl)
			3'b00?: Result = Sum; // ALUControl signal 00/01 means ADD and SUB, so either way A and B will be added
			3'b010: Result = A & B; // ALUControl signal 10 means AND
			3'b011: Result = A | B; // ALUControl signal 01 means OR
            //MUL: 100
            //SMULL: 101
            //UMULL: 111 voy a hacer push y cortar el live share 
            3'b1??: {ResultExtra, Result} = A*B;
		endcase
	end

  assign Overflow = ~(ALUControl[0]^A[31]^B[31]) & (Sum[31]^A[31]) & ~ALUControl[1]; // Overflow flag wire
	assign Zero = 32'b0 == Result; // If every bit is 0, then the zero result flag will activate
	assign Negative = Result[31]&~ALUControl[2] | ResultExtra[31]&ALUControl[2]; // If the first bit from the left is 1,  the result will be negative
	assign Carry = ~ALUControl[1] & Sum[32]; // Carry will be 1 if the ALUControl signal is addition or subtraction and if the 1st bit from the left bit is 1

	assign ALUFlags = {Negative, Zero, Carry, Overflow}; // We group each bit-flag to the ALUFlags output    

endmodule