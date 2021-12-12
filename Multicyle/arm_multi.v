// arm_multi.v
// David_Harris@hmc.edu, Sarah_Harris@hmc.edu 25 December 2013
// Multi-cycle implementation of a subset of ARMv4

// 16 32-bit registers
// Data-processing instructions
//   ADD, SUB, AND, ORR
//   INSTR <cond> <S> <Rd>, <Rn>, #immediate
//   INSTR <cond> <S> <Rd>, <Rn>, <Rm>
//    Rd <- <Rn> INSTR <Rm>	    	if (S) Update Status Flags
//    Rd <- <Rn> INSTR immediate	if (S) Update Status Flags
//   Instr[31:28] = cond
//   Instr[27:26] = Op = 00
//   Instr[25:20] = Funct
//                  [25]:    1 for immediate, 0 for register
//                  [24:21]: 0100 (ADD) / 0010 (SUB) /
//                           0000 (AND) / 1100 (ORR)
//                  [20]:    S (1 = update CPSR status Flags)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:8]  = 0000
//   Instr[7:0]   = immed_8  (for #immediate type) / 
//                  0000<Rm> (for register type)
//   
// Load/Store instructions
//   LDR, STR
//   INSTR <Rd>, [<Rn>, #offset]
//    LDR: Rd <- Mem[<Rn>+offset]
//    STR: Mem[<Rn>+offset] <- Rd
//   Instr[31:28] = cond
//   Instr[27:26] = Op = 01 
//   Instr[25:20] = Funct
//                  [25]:    0 (A)
//                  [24:21]: 1100 (P/U/B/W)
//                  [20]:    L (1 for LDR, 0 for STR)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:0]  = imm (zero extended)
//
// Branch instruction (PC <= PC + offset, PC holds 8 bytes past Branch
//   B
//   INSTR <target>
//    PC <- PC + 8 + imm << 2
//   Instr[31:28] = cond
//   Instr[27:25] = Op = 10
//   Instr[25:24] = Funct
//                  [25]: 1 (Branch)
//                  [24]: 0 (link)
//   Instr[23:0]  = offset (sign extend, shift left 2)
//   Note: no Branch delay slot on ARM
//
// Other:
//   R15 reads as PC+8
//   Conditional Encoding
//    cond  Meaning                       Flag
//    0000  Equal                         Z = 1
//    0001  Not Equal                     Z = 0
//    0010  Carry Set                     C = 1
//    0011  Carry Clear                   C = 0
//    0100  Minus                         N = 1
//    0101  Plus                          N = 0
//    0110  Overflow                      V = 1
//    0111  No Overflow                   V = 0
//    1000  Unsigned Higher               C = 1 & Z = 0
//    1001  Unsigned Lower/Same           C = 0 | Z = 1
//    1010  Signed greater/equal          N = V
//    1011  Signed less                   N != V
//    1100  Signed greater                N = V & Z = 0
//    1101  Signed less/equal             N != V | Z = 1
//    1110  Always                        any
//   Writes to register 15 (PC) are ignored 
`include "../ALU.v"

module top (
	clk,
	reset,
	WriteData,
	Adr,
	MemWrite
);
	input wire clk;
	input wire reset;
	output wire [31:0] WriteData;
	output wire [31:0] Adr;
	output wire MemWrite;
	wire [31:0] PC;
	wire [31:0] Instr;
	wire [31:0] ReadData;
	// instantiate processor and shared memory
	arm arm(
		.clk(clk),
		.reset(reset),
		.MemWrite(MemWrite),
		.Adr(Adr),
		.Instr(Instr),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
	mem mem(
		.clk(clk),
		.we(MemWrite),
		.a(Adr),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule
module mem (
	clk,
	we,
	a,
	wd,
	rd
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	initial $readmemh("memfile.dat", RAM);
	assign rd = RAM[a[31:2]]; // word aligned
	always @(posedge clk)
		if (we)
			RAM[a[31:2]] <= wd;
endmodule

module arm (
	clk,
	reset,
	MemWrite,
	Adr,
	Instr,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	output wire MemWrite;
	output wire [31:0] Adr;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	input wire [31:0] Instr;
	wire [3:0] ALUFlags;
	wire PCWrite;
	wire RegWrite;
	wire IRWrite;
	wire AdrSrc;
	wire [1:0] RegSrc;
	wire [1:0] ALUSrcA;
	wire [1:0] ALUSrcB;
	wire [1:0] ImmSrc;
	wire [2:0] ALUControl;
	wire [1:0] ResultSrc;
	controller c(
		.clk(clk),
		.reset(reset),
		.Instr(Instr[31:12]),
		.ALUFlags(ALUFlags),
		.PCWrite(PCWrite),
		.MemWrite(MemWrite),
		.RegWrite(RegWrite),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.RegSrc(RegSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.ImmSrc(ImmSrc),
		.ALUControl(ALUControl)
	);
	datapath dp(
		.clk(clk),
		.reset(reset),
		.Adr(Adr),
		.WriteData(WriteData),
		.ReadData(ReadData),
		.Instr(Instr),
		.ALUFlags(ALUFlags),
		.PCWrite(PCWrite),
		.RegWrite(RegWrite),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.RegSrc(RegSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.ImmSrc(ImmSrc),
		.ALUControl(ALUControl)
	);
endmodule

module controller (
	clk,
	reset,
	Instr,
	ALUFlags,
	PCWrite,
	MemWrite,
	RegWrite,
	IRWrite,
	AdrSrc,
	RegSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	ImmSrc,
	ALUControl
);
	input wire clk;
	input wire reset;
	input wire [31:12] Instr;
	input wire [3:0] ALUFlags;
	output wire PCWrite;
	output wire MemWrite;
	output wire RegWrite;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] RegSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ResultSrc;
	output wire [1:0] ImmSrc;
	output wire [2:0] ALUControl;
	wire [1:0] FlagW;
	wire PCS;
	wire NextPC;
	wire RegW;
	wire MemW;
	decode dec(
		.clk(clk),
		.reset(reset),
		.Op(Instr[27:26]),
		.Funct(Instr[25:20]),
		.Rd(Instr[15:12]),
		.FlagW(FlagW),
		.PCS(PCS),
		.NextPC(NextPC),
		.RegW(RegW),
		.MemW(MemW),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.ResultSrc(ResultSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ImmSrc(ImmSrc),
		.RegSrc(RegSrc),
		.ALUControl(ALUControl)
	);
	condlogic cl(
		.clk(clk),
		.reset(reset),
		.Cond(Instr[31:28]),
		.ALUFlags(ALUFlags),
		.FlagW(FlagW),
		.PCS(PCS),
		.NextPC(NextPC),
		.RegW(RegW),
		.MemW(MemW),
		.PCWrite(PCWrite),
		.RegWrite(RegWrite),
		.MemWrite(MemWrite)
	);
endmodule

module decode (
	clk,
	reset,
	Op,
	Funct,
	Rd,
	FlagW,
	PCS,
	NextPC,
	RegW,
	MemW,
	IRWrite,
	AdrSrc,
	ResultSrc,
	ALUSrcA,
	ALUSrcB,
	ImmSrc,
	RegSrc,
	ALUControl
);
	input wire clk;
	input wire reset;
	input wire [1:0] Op;
	input wire [5:0] Funct;
	input wire [3:0] Rd;
	output wire [1:0] FlagW;
	output wire PCS;
	output wire NextPC;
	output wire RegW;
	output wire MemW;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] ResultSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ImmSrc;
	output wire [1:0] RegSrc;
	output wire [2:0] ALUControl;
	wire Branch;
	wire ALUOp;

	// Main FSM
	mainfsm fsm(
		.clk(clk),
		.reset(reset),
		.Op(Op),
		.Funct(Funct),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.NextPC(NextPC),
		.RegW(RegW),
		.MemW(MemW),
		.Branch(Branch),
		.ALUOp(ALUOp)
	);

	// ADD CODE BELOW
	// Add code for the ALU Decoder and PC Logic.
	// Remember, you may reuse code from previous labs.
	// ALU Decoder

	// PC Logic


	// Add code for the Instruction Decoder (Instr Decoder) below.
	// Recall that the input to Instr Decoder is Op, and the outputs are
	// ImmSrc and RegSrc. We've completed the ImmSrc logic for you.

	// Instr Decoder
	// ALU Decoder 
	always @(*) begin
		if (ALUOp) begin
			case (Funct[4:1])
				4'b0100: ALUControl = 3'b000;
				4'b0010: ALUControl = 3'b001;
				4'b0000: ALUControl = 3'b010;
				4'b1100: ALUControl = 3'b011;
				4'b1000: ALUControl = 3'b100;
				default: ALUControl = 3'bxxx;
			endcase
			FlagW[1] <= Funct[0];
			FlagW[0] <= Funct[0] & ((ALUControl == 3'b000) | (ALUControl == 3'b001));
		end 
		else begin
			ALUControl = 3'b00;
			FlagW = 2'b00;
		end
	end
	// PC Controller
	assign PCS = ((Rd == 4'b1111) & RegW) | Branch;

	// Instr Decoder
	assign ImmSrc = Op;

	// RegSrc Decoder
	assign RegSrc[0] = (Op == 2'b01);
	assign RegSrc[1] = (Op == 2'b10);
	assign ImmSrc = Op;
endmodule

module mainfsm (
	clk,
	reset,
	Op,
	Funct,
	IRWrite,
	AdrSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	NextPC,
	RegW,
	MemW,
	Branch,
	ALUOp
);
	input wire clk;
	input wire reset;
	input wire [1:0] Op;
	input wire [5:0] Funct;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ResultSrc;
	output wire NextPC;
	output wire RegW;
	output wire MemW;
	output wire Branch;
	output wire ALUOp;
	reg [3:0] state;
	reg [3:0] nextstate;
	reg [12:0] controls;
	localparam [3:0] FETCH = 0;
	localparam [3:0] BRANCH = 9;
	localparam [3:0] DECODE = 1;
	localparam [3:0] EXECUTEI = 7;
	localparam [3:0] EXECUTER = 6;
	localparam [3:0] MEMADR = 2;
	localparam [3:0] UNKNOWN = 10;

	// state register
	always @(posedge clk or posedge reset)
		if (reset)
			state <= FETCH;
		else
			state <= nextstate;
	

	// ADD CODE BELOW
  	// Finish entering the next state logic below.  We've completed the 
  	// first two states, FETCH and DECODE, for you.

  	// next state logic
	always @(*)
		casex (state)
			FETCH: nextstate = DECODE;
			DECODE:
				case (Op)
					2'b00:
						if (Funct[5])
							nextstate = EXECUTEI;
						else
							nextstate = EXECUTER;
					2'b01: nextstate = MEMADR;
					2'b10: nextstate = BRANCH;
					default: nextstate = UNKNOWN;
				endcase
			EXECUTER: nextstate = ALUWB;
			EXECUTEI: nextstate = ALUWB;
			MEMADR:
				case (Funct[0])
					1'b1: nextstate = MEMRD;
					1'b0: nextstate = MEMWR;
				endcase
			MEMRD: nextstate = MEMWB;
			default: nextstate = FETCH;
		endcase

	// ADD CODE BELOW
	// Finish entering the output logic below.  We've entered the
	// output logic for the first two states, FETCH and DECODE, for you.

	// state-dependent output logic
	always @(*)
		case (state)
			FETCH: controls = 13'b1000101001100;
			DECODE: controls = 13'b0000001001100;
			EXECUTER: controls = 13'b0000000000001;
            EXECUTEI: controls = 13'b0000000000011;
            ALUWB: controls = 13'b0001000000000;
            MEMADR: controls = 13'b0000000000010;
            MEMWR: controls = 13'b0010010000000;
            MEMRD: controls = 13'b0000010000000;
            MEMWB: controls = 13'b0001000100000;
            BRANCH: controls = 13'b0100001000010;
			default: controls = 13'bxxxxxxxxxxxxx;
		endcase
	assign {NextPC, Branch, MemW, RegW, IRWrite, AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, ALUOp} = controls;
endmodule

// el esta haciendo ya multiplicadores y lo de fp add

// ADD CODE BELOW
// Add code for the condlogic and condcheck modules. Remember, you may
// reuse code from prior labs.
module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	NextPC,
	RegW,
	MemW,
	PCWrite,
	RegWrite,
	MemWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire NextPC;
	input wire RegW;
	input wire MemW;
	output wire PCWrite;
	output wire RegWrite;
	output wire MemWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;

	// Delay writing flags until ALUWB state
	flopr #(2) flagwritereg(
		clk,
		reset,
		FlagW & {2 {CondEx}},
		FlagWrite
	);

	// ADD CODE HERE

	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);
	
	// condcheck cc(
	// 	.Cond(Cond),
	// 	.Flags(Flags),
	// 	.CondEx(CondEx)
	// );
	// always @(posedge clk, negedge reset) begin
	// 	if (reset) begin
	// 		Flags[3:0] <= 4'b0000;
	// 	end
	// 	else begin
	// 		if (FlagWrite[1]) begin
	// 			Flags[3:2] <= ALUFlags[3:2];
	// 		end
	// 		if (FlagWrite[0]) begin
	// 			Flags[1:0] <= ALUFlags[1:0];
	// 		end
	// 	end
	// end
		
	// always @(posedge clk, negedge reset) begin
	// 	if (reset) begin
	// 		CondEx <= 0;
	// 	end
	// 	else begin
	// 		CondEx <= CondEx;
	// 	end
	// end

	// assign FlagWrite = (CondEx) ? FlagW : 2'b00;
	// always begin
	// 	assign RegWrite = RegW & CondEx;
	// 	assign MemWrite = MemW & CondEx;
	// end
	// assign PCWrite = NextPC || (PCWrite && CondEx);

endmodule

module condcheck (
	Cond,
	Flags,
	CondEx
);
	input wire [3:0] Cond;
	input wire [3:0] Flags;
	output reg CondEx;
	wire neg;
	wire zero;
	wire carry;
	wire overflow;
	wire ge;

	assign {neg, zero, carry, overflow} = Flags;
	assign ge = neg == overflow;
	always @(*)
		case (Cond)
			4'b0000: CondEx = zero;
			4'b0001: CondEx = ~zero;
			4'b0010: CondEx = carry;
			4'b0011: CondEx = ~carry;
			4'b0100: CondEx = neg;
			4'b0101: CondEx = ~neg;
			4'b0110: CondEx = overflow;
			4'b0111: CondEx = ~overflow;
			4'b1000: CondEx = carry & ~zero;
			4'b1001: CondEx = ~(carry & ~zero);
			4'b1010: CondEx = ge;
			4'b1011: CondEx = ~ge;
			4'b1100: CondEx = ~zero & ge;
			4'b1101: CondEx = ~(~zero & ge);
			4'b1110: CondEx = 1'b1;
			default: CondEx = 1'bx;
		endcase
endmodule

// ADD CODE BELOW
// Complete the datapath module below for Lab 11.
// You do not need to complete this module for Lab 10.
// The datapath unit is a structural SystemVerilog module. That is,
// it is composed of instances of its sub-modules. For example,
// the instruction register is instantiated as a 32-bit flopenr.
// The other submodules are likewise instantiated. 

module datapath (
	clk,
	reset,
	Adr,
	WriteData,
	ReadData,
	Instr,
	ALUFlags,
	PCWrite,
	RegWrite,
	IRWrite,
	AdrSrc,
	RegSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	ImmSrc,
	ALUControl
);
	input wire clk;
	input wire reset;
	output wire [31:0] Adr;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	output wire [31:0] Instr;
	// Register handling
	// if (Instr[25] == 1'b0 & Instr[7:4] == 4'b1001) begin

	if (1) begin
		regfile mulregfile(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.cmd(Instr[23:21]),
			.S(Instr[20]),
			.Rd(Intr[19:16]),
			.Ra(Instr[15:12]),
			.Rm(Instr[11:8]),
			.Rn(Instr[3:0])
		);
	end
	
	else if (Instr[27:26] == 2'b00) begin
		regfile regDataProcessing(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.I(Instr[25]),
			.cmd(Instr[24:21]),
			.S(Instr[20]),
			.Rn(Instr[19:16]),
			.Rd(Instr[15:12]),
			.Src2(Instr[11:0])
		);
	end

	else if (Instr[27:26] != 2'b00 & Instr[6:5] != 2'b00) begin
		regfile memoryInst(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.Funct(Instr[25:20]),
			.Rn(Intr[19:16]),
			.Rd(Instr[15:12]),
			.Src2(Instr[11:0]),
			.Mem(WriteData),
			.Adr(Adr)
		);
	end
	else if (Instr[23:20] != 2'b00) begin
		regfile regInstBranch(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.Funct(Instr[25:20]),
			.Rn(Intr[19:16]),
			.Rd(Instr[15:12]),
			.Src2(Instr[11:0])
		);
	end
	else if (Instr[27:26] == 2'b10) begin
		regfile regInstBranch(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.funct(Instr[25:24]),
			.immd(Instr[23:0])
		);
	end
	output wire [3:0] ALUFlags;
	input wire PCWrite;
	input wire RegWrite;
	input wire IRWrite;
	input wire AdrSrc;
	input wire [1:0] RegSrc;
	input wire [1:0] ALUSrcA;
	input wire [1:0] ALUSrcB;
	input wire [1:0] ResultSrc;
	input wire [1:0] ImmSrc;
	input wire [2:0] ALUControl;
	wire [31:0] PCNext;
	wire [31:0] PC;
	wire [31:0] ExtImm;
	wire [31:0] SrcA;
	wire [31:0] SrcB;
	wire [31:0] Result;
	wire [31:0] Data;
	wire [31:0] RD1;
	wire [31:0] RD2;
	wire [31:0] A;
	wire [31:0] ALUResult;
	wire [31:0] ALUResultExtra;
	wire [31:0] ALUOut;
	wire [3:0] RA1;
	wire [3:0] RA2;

	// Your datapath hardware goes below. Instantiate each of the 
	// submodules that you need. Remember that you can reuse hardware
	// from previous labs. Be sure to give your instantiated modules 
	// applicable names such as pcreg (PC register), adrmux 
	// (Address Mux), etc. so that your code is easier to understand.

	// ADD CODE HERE
	// Falta añadir el código para el datapath

	
	always @(posedge clk) begin
		if(PCWrite) begin 
			PC <= Result;
		end
	end

	assign Adr = (AdrSrc?Result:PC);

	always @(posedge clk) begin
		Data <= ReadData; 
		Instr <= ReadData; 
	end
	
	wire [31:0] rdata1, rdata2;
	//register_file rfile(clk, RA1, RA2, Instr[15:12], Result, Result, RegWrite, rdata1, rdata2);

	always @(posedge clk) begin
		A <= rdata1;
		WriteData <= rdata2;
	end

	assign SrcA = (ALUSrcA?PC:A);
	
	extend ext(Instr[23:0], ImmSrc, ExtImm);

	mux3 alusrcb(WriteData, ExtImm, 4, ALUSrcB, SrcB);

	alu alu_dp(
		.A(SrcA),
		.B(SrcB), 
		.ALUControl(ALUControl),
		.ALUResult(ALUResult),
		.ALUResultExtra(ALUResultExtra),
		.ALUFlags(ALUFlags)
		);

	always @(posedge clk) begin
		ALUOut <= ALUResult;
	end

	mux3 muxresult(ALUOut, Data, ALUResult, ResultSrc, Result);

endmodule


module register_file(
	input wire clk,
	input wire [3:0] A1, input wire [3:0] A2, input wire [3:0] A3,
	input wire [31:0] WD3, input wire [31:0] R15, input wire WE3,
	output wire [31:0] RD1, output wire [31:0] RD2
);
	reg [31:0] registros [14:0];
	integer i;
	initial begin
		for( i= 0; i<15; i++) begin
		  registros[i] = 32'b0;
		end
	end

	assign RD1 = registros[A1];
	assign RD2 = registros[A2];

	always @(posedge clk) begin
		if (WE3) begin
			registros[A3] <= WD3;
		end
	end

endmodule


module extend (
	input wire [23:0] Instr,
	input wire [1:0] ImmSrc,
	output reg [31:0] ExtImm
);
	always @(*) begin
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
	end
endmodule

module adder (
	a,
	b,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] a;
	input wire [WIDTH - 1:0] b;
	output wire [WIDTH - 1:0] y;
	assign y = a + b;
endmodule


module flopenr (
	clk,
	reset,
	en,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset) begin
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
	end
endmodule


module flopr (
	clk,
	reset,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset) begin
		if (reset)
			q <= 0;
		else
			q <= d;
	end
endmodule

module mux2 (
	d0,
	d1,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire s;
	output wire [WIDTH - 1:0] y;
	assign y = (s ? d1 : d0);
endmodule

module mux3 (
	d0,
	d1,
	d2,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire [WIDTH - 1:0] d2;
	input wire [1:0] s;
	output wire [WIDTH - 1:0] y;
	assign y = (s[1] ? d2 : (s[0] ? d1 : d0));
endmodule

// Register file para mem
module mulregfile (
	cond,
	Op,
	cmd,
	S,
	Rd,
	Ra,
	Rn,
	Rm
);
	input wire cond[3:0];
	input wire Op[1:0];
	input wire cmd[2:0];
	input wire S;
	output wire Rd[3:0];
	output wire Ra[3:0];
	output wire Rn[3:0];
	output wire Rm[3:0];

	always @(*) begin
		case (cmd)
			3'b000: // Rd ← Rn × Rm (low 32 bits)			Multiply
				Rd <= Rn & Rm;
			3'b001: // Rd ← (Rn × Rm)+Ra (low 32 bits)		Multiply accumulate
				Rd <= (Rn & Rm) | Ra;
			3'b100: {Rd, Ra} <= Rn & Rm;
					// {Rd, Ra} ← Rn × Rm					Unsigned Multiply Long
					// (all 64 bits, Rm/Rn unsigned)
			3'b101: // {Rd, Ra} ← (Rn × Rm)+{Rd, Ra}		Unsigned Multiply accumulate Long
					// (all 64 bits, Rm/Rn unsigned)
				{Rd, Ra} <= (Rn & Rm) | {Rd, Ra};
			3'b110: // {Rd, Ra} ← Rn × Rm					Signed Multiply Long
					// (all 64 bits, Rm/Rn signed)
				{Rd, Ra} <= Rn & Rm;
			3'b111: // {Rd, Ra} ← (Rn × Rm)+{Rd, Ra}		Signed Multiply accumulate Long
					// (all 64 bits, Rm/Rn signed)
				{Rd, Ra} <= (Rn & Rm) | {Rd, Ra};
			default: // invalido
				Rd <= Rn & Rm;							  // De todas formas lo mando a Multiply
		endcase
	end
endmodule


// Register file para multiplicacion
module memoryInst (
	cond,
	Op,
	Funct,
	Rn,
	Rd,
	Src2,
	Mem,
	Adr
);
	input wire cond[3:0];
	input wire Op[1:0];
	input wire Funct[6:0];
	input wire Adr[31:0];
	wire L;
	output wire Rn[3:0];
	output wire Rd[3:0];
	output wire Mem[31:0];
	wire Src2a[3:0];
	wire Src2b[3:0];
	wire op2[1:0];

	assign L = Funct[0];
	assign Src2a = Src2[11:8];
	assign Src2b = Src2[3:0];
	assign op2 = Src2[6:5];

	always @(*) begin
		case(Op)
			2'b01:
				if (Funct[2] == 1'b0) begin
					if (L == 1'b0) begin
						// Mem[Adr] ← Rd
						Mem[Adr] <= Rd;
					end
					if (L == 1'b1) begin
						// Rd ← Mem[Adr]
						Rd <= Mem[Adr];
					end
				end
				else if (Funct[2] == 1'b1) begin
					if (L == 1'b0) begin
						// Mem[Adr] ← Rd7:0
						Mem[Adr] <= Rd[7:0];
					end
					if (L == 1'b1) begin
						// Rd ← Mem[Adr]7:0
						Rd[7:0] <= Mem[Adr];
					end
				end
			2'b00: 
				if (op2 == 2'b01) begin
						if (L == 1'b0) begin
							// Mem[Adr] ← Rd15:0
							Mem[Adr] <= Rd[15:0];
						end
						if (L == 1'b1) begin
							// Rd ← Mem[Adr]15:0
							Rd <= Mem[15:0];
						end
					end
				else if (op2 == 2'b10) begin
					if (L == 1'b1) begin
						// Rd ← Mem[Adr]7:0
						Rd <= Mem[7:0];
					end
				end
				else if (op2 == 2'b11) begin
					if (L == 1'b1) begin
						// Rd ← Mem[Adr]15:0
						Rd <= Mem[15:0];
					end
				end
			default:
				Mem[Adr] <= Rd;
		endcase
	end


endmodule

// register file, branching

module regInstBranch (
	cond,
	Op,
	funct,
	immd,
);
	input wire cond[3:0];
	input wire Op[1:0];
	input wire funct[1:0];
	input wire immd[23:0];
	wire PC[31:0]; 
	wire LR[31:0];

	always @(*) begin
		if (funct == 2'b00) begin
			// PC ← (PC+8)+imm24 << 2
		end
		else if (funct != 2'b01) begin
			// LR ← (PC+8) – 4; PC ← (PC+8)+imm24 << 2
		end
	end


endmodule

// register file for data processing
module regDataProcessing (
	cond,
	Op,
	I,
	cmd,
	S,
	Rn,
	Rd,
	Src2
);
	input wire cond[3:0];
	input wire Op[1:0];
	input wire I;
	input wire cmd[2:0];
	input wire Src2[11:0];
	input S;
	output wire Rn[3:0];
	output wire Rd[3:0];

	always @(*) begin
		case (cmd)
			4'b0000: Rd <= Rn & Src2;
			4'b0001: Rd <= Rn ^ Src2;
			4'b0010: Rd <= Rn - Src2;
			4'b0011: Rd <= Src2 - Rn;
			4'b0100: Rd <= Rn + Src2;
			4'b1000: 
				if (S == 1'b1) begin
					//Set flags based on Rn & Src2
				end
			4'b1001:
				if (S == 1'b1) begin
					//Set flags based on Rn ^ Src2
				end
			4'b1010:
				if (S == 1'b1) begin
					//Set flags based on Rn - Src2
				end
			4'b1011:
				if (S == 1'b1) begin
					//Set flags based on Src2 - Rn
				end
			4'b1100: Rd <= Rn | Src2;
			4'b1101:
				if (I == 1'b1 & Src2[11:4] == 8'b0) begin
					
				end
			4'b1110: Rd <= Rn & Src2;
			4'b1111: Rd <= Rn & Src2;
			default:
				Rd <= Rn & Src2;
		endcase
	end

endmodule
