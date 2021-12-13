module alu (input [31:0] a,b,
             input [2:0] ALUControl,    
             output reg [31:0] Result32, //assign always block
             output reg [31:0] Result64,
             output wire [3:0] ALUFlags); //explicit wire for assign with {}
  
  wire negative, zero, carry, overflow; // define wire for each flag (n,z,c,v)
  wire [32:0] sum;
  
  
  assign sum = a + (ALUControl[0]? ~b: b) + ALUControl[0]; //ADDER: two's complement
  
  /*
  ALUControl Logic
  00?: sum
  001: sub
  010: and
  011: or
  101: MUL
  110: UMULL
  111: SMULL
  */
  
  always @(*)
    casex (ALUControl[1:0]) //case, casex, casez
      3'b00?: Result32 = sum;    // ADD
      3'b001: Result32 = a - b;  // SUB
      3'b010: Result32 = a & b;  // AND
      3'b011: Result32 = a | b;  // OR
      3'b101: Result32 = a * b;  // MUL
      3'b110:                    // SMULL
              begin
                {Result64, Result32} = a * b;  
              end 
      3'b111:                    // UMULL 
              begin
                if(a[31] == 1 && b[31] == 1) begin
                  {Result64, Result32} = (-a * -b);
                end
                else if(a[31] == 1 || b[31] == 1) begin
                  {Result64, Result32} = -(a * b);
                end
                else begin
                  {Result64, Result32} = (a * b);
                end
              end       
    endcase
  
 //flags: result -> negative, zero
  assign negative = Result32[31];
  assign zero = (Result32 == 32'b0);
  //flags: additional logic -> v, c
  assign carry = (ALUControl[1]==1'b0) & sum[32];
  assign overflow = (ALUControl[1]==1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);

  assign ALUFlags = {negative, zero, carry, overflow};

endmodule