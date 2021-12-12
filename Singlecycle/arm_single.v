module testbench;
	reg clk;
	reg reset;
	wire [31:0] WriteData;
	wire [31:0] DataAdr;
	wire MemWrite;
	top dut(
		.clk(clk),
		.reset(reset),
		.WriteData(WriteData),
		.DataAdr(DataAdr),
		.MemWrite(MemWrite)
	);
	initial begin
		reset <= 1;
		#(22)
			;
		reset <= 0;
	end
	always begin
		clk <= 1;
		#(5)
			;
		clk <= 0;
		#(5)
			;
	end
	always @(negedge clk)
		if (MemWrite)
			if ((DataAdr === 100) & (WriteData === 7)) begin
				$display("Simulation succeeded");
				$stop;
			end
			else if (DataAdr !== 96) begin
				$display("Simulation failed");
				$stop;
			end
endmodule
module top (
	clk,
	reset,
	WriteData,
	DataAdr,
	MemWrite
);
	input wire clk;
	input wire reset;
	output wire [31:0] WriteData;
	output wire [31:0] DataAdr;
	output wire MemWrite;
	wire [31:0] PC;
	wire [31:0] Instr;
	wire [31:0] ReadData;
	arm arm(
		.clk(clk),
		.reset(reset),
		.PC(PC),
		.Instr(Instr),
		.MemWrite(MemWrite),
		.ALUResult(DataAdr),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
	imem imem(
		.a(PC),
		.rd(Instr)
	);
	dmem dmem(
		.clk(clk),
		.we(MemWrite),
		.a(DataAdr),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule
module dmem (
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
	assign rd = RAM[a[31:2]];
	always @(posedge clk)
		if (we)
			RAM[a[31:2]] <= wd;
endmodule
module imem (
	a,
	rd
);
	input wire [31:0] a;
	output wire [31:0] rd;
	wire [31:0] RAM [63:0];
	initial $readmemh("memfile.dat", RAM);
	assign rd = RAM[a[31:2]];
endmodule
module arm (
	clk,
	reset,
	PC,
	Instr,
	MemWrite,
	ALUResult,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	output wire [31:0] PC;
	input wire [31:0] Instr;
	output wire MemWrite;
	output wire [31:0] ALUResult;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	wire [3:0] ALUFlags;
	wire RegWrite;
	wire ALUSrc;
	wire MemtoReg;
	wire PCSrc;
	wire [1:0] RegSrc;
	wire [1:0] ImmSrc;
	wire [1:0] ALUControl;
	controller c(
		.clk(clk),
		.reset(reset),
		.Instr(Instr[31:12]),
		.ALUFlags(ALUFlags),
		.RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.ImmSrc(ImmSrc),
		.ALUSrc(ALUSrc),
		.ALUControl(ALUControl),
		.MemWrite(MemWrite),
		.MemtoReg(MemtoReg),
		.PCSrc(PCSrc)
	);
	datapath dp(
		.clk(clk),
		.reset(reset),
		.RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.ImmSrc(ImmSrc),
		.ALUSrc(ALUSrc),
		.ALUControl(ALUControl),
		.MemtoReg(MemtoReg),
		.PCSrc(PCSrc),
		.ALUFlags(ALUFlags),
		.PC(PC),
		.Instr(Instr),
		.ALUResult(ALUResult),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
endmodule
module controller (
	clk,
	reset,
	Instr,
	ALUFlags,
	RegSrc,
	RegWrite,
	ImmSrc,
	ALUSrc,
	ALUControl,
	MemWrite,
	MemtoReg,
	PCSrc
);
	input wire clk;
	input wire reset;
	input wire [31:12] Instr;
	input wire [3:0] ALUFlags;
	output wire [1:0] RegSrc;
	output wire RegWrite;
	output wire [1:0] ImmSrc;
	output wire ALUSrc;
	output wire [1:0] ALUControl;
	output wire MemWrite;
	output wire MemtoReg;
	output wire PCSrc;
	wire [1:0] FlagW;
	wire PCS;
	wire RegW;
	wire MemW;
	decode dec(
		.Op(Instr[27:26]),
		.Funct(Instr[25:20]),
		.Rd(Instr[15:12]),
		.FlagW(FlagW),
		.PCS(PCS),
		.RegW(RegW),
		.MemW(MemW),
		.MemtoReg(MemtoReg),
		.ALUSrc(ALUSrc),
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
		.RegW(RegW),
		.MemW(MemW),
		.PCSrc(PCSrc),
		.RegWrite(RegWrite),
		.MemWrite(MemWrite)
	);
endmodule
module decode (
	Op,
	Funct,
	Rd,
	FlagW,
	PCS,
	RegW,
	MemW,
	MemtoReg,
	ALUSrc,
	ImmSrc,
	RegSrc,
	ALUControl
);
	input wire [1:0] Op;
	input wire [5:0] Funct;
	input wire [3:0] Rd;
	output reg [1:0] FlagW;
	output wire PCS;
	output wire RegW;
	output wire MemW;
	output wire MemtoReg;
	output wire ALUSrc;
	output wire [1:0] ImmSrc;
	output wire [1:0] RegSrc;
	output reg [1:0] ALUControl;
	reg [9:0] controls;
	wire Branch;
	wire ALUOp;
	always @(*)
		casex (Op)
			2'b00:
				if (Funct[5])
					controls = 10'b0000101001;
				else
					controls = 10'b0000001001;
			2'b01:
				if (Funct[0])
					controls = 10'b0001111000;
				else
					controls = 10'b1001110100;
			2'b10: controls = 10'b0110100010;
			default: controls = 10'bxxxxxxxxxx;
		endcase
	assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, RegW, MemW, Branch, ALUOp} = controls;
	always @(*)
		if (ALUOp) begin
			case (Funct[4:1])
				4'b0100: ALUControl = 2'b00;
				4'b0010: ALUControl = 2'b01;
				4'b0000: ALUControl = 2'b10;
				4'b1100: ALUControl = 2'b11;
				default: ALUControl = 2'bxx;
			endcase
			FlagW[1] = Funct[0];
			FlagW[0] = Funct[0] & ((ALUControl == 2'b00) | (ALUControl == 2'b01));
		end
		else begin
			ALUControl = 2'b00;
			FlagW = 2'b00;
		end
	assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule
module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	RegW,
	MemW,
	PCSrc,
	RegWrite,
	MemWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire RegW;
	input wire MemW;
	output wire PCSrc;
	output wire RegWrite;
	output wire MemWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;
	flopenr #(2) flagreg1(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])
	);
	flopenr #(2) flagreg0(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
	);
	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);
	assign FlagWrite = FlagW & {2 {CondEx}};
	assign RegWrite = RegW & CondEx;
	assign MemWrite = MemW & CondEx;
	assign PCSrc = PCS & CondEx;
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
module datapath (
	clk,
	reset,
	RegSrc,
	RegWrite,
	ImmSrc,
	ALUSrc,
	ALUControl,
	MemtoReg,
	PCSrc,
	ALUFlags,
	PC,
	Instr,
	ALUResult,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	input wire [1:0] RegSrc;
	input wire RegWrite;
	input wire [1:0] ImmSrc;
	input wire ALUSrc;
	input wire [1:0] ALUControl;
	input wire MemtoReg;
	input wire PCSrc;
	output wire [3:0] ALUFlags;
	output wire [31:0] PC;
	input wire [31:0] Instr;

	// Register handling
	if (Instr[25] == 1'b0 & Instr[7:4] == 4'b1001) begin
		mulregfile regfile(
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
		regDataProcessing regfile(
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
		memoryInst regfile(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.Funct(Instr[25:20])
			.Rn(Intr[19:16]),
			.Rd(Instr[15:12]),
			.Src2(Instr[11:0]),
			.Mem(WriteData),
			.Adr(Adr)
		);
	end
	else if (Instr[23:20] != 2'b00) begin
		regInstBranch regfile(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.Funct(Instr[25:20])
			.Rn(Intr[19:16]),
			.Rd(Instr[15:12]),
			.Src2(Instr[11:0])
		);
	end
	else if (Instr[27:26] == 2'b10) begin
		regDataProcessing regfile(
			.cond(Instr[31:28]),
			.Op(Instr[27:26]),
			.funct(Instr[25:24]),
			.immd(Instr[23:0])
		);
	end

	output wire [31:0] ALUResult;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	wire [31:0] PCNext;
	wire [31:0] PCPlus4;
	wire [31:0] PCPlus8;
	wire [31:0] ExtImm;
	wire [31:0] SrcA;
	wire [31:0] SrcB;
	wire [31:0] Result;
	wire [3:0] RA1;
	wire [3:0] RA2;
	mux2 #(32) pcmux(
		.d0(PCPlus4),
		.d1(Result),
		.s(PCSrc),
		.y(PCNext)
	);
	flopr #(32) pcreg(
		.clk(clk),
		.reset(reset),
		.d(PCNext),
		.q(PC)
	);
	adder #(32) pcadd1(
		.a(PC),
		.b(32'b100),
		.y(PCPlus4)
	);
	adder #(32) pcadd2(
		.a(PCPlus4),
		.b(32'b100),
		.y(PCPlus8)
	);
	mux2 #(4) ra1mux(
		.d0(Instr[19:16]),
		.d1(4'b1111),
		.s(RegSrc[0]),
		.y(RA1)
	);
	mux2 #(4) ra2mux(
		.d0(Instr[3:0]),
		.d1(Instr[15:12]),
		.s(RegSrc[1]),
		.y(RA2)
	);
	mux2 #(32) resmux(
		.d0(ALUResult),
		.d1(ReadData),
		.s(MemtoReg),
		.y(Result)
	);
	extend ext(
		.Instr(Instr[23:0]),
		.ImmSrc(ImmSrc),
		.ExtImm(ExtImm)
	);
	mux2 #(32) srcbmux(
		.d0(WriteData),
		.d1(ExtImm),
		.s(ALUSrc),
		.y(SrcB)
	);
	alu alu(
		SrcA,
		SrcB,
		ALUControl,
		ALUResult,
		ALUFlags
	);
endmodule

module extend (
	Instr,
	ImmSrc,
	ExtImm
);
	input wire [23:0] Instr;
	input wire [1:0] ImmSrc;
	output reg [31:0] ExtImm;
	always @(*)
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
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
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
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
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else
			q <= d;
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

// Register file para mem
module mulregfile(
	cond,
	Op,
	cmd,
	S,
	Rd,
	Ra,
	Rn,
	Rm
)
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
				Rd <= (Rn & Rm) | Ra
			3'b100: // {Rd, Ra} ← Rn × Rm					Unsigned Multiply Long
					// (all 64 bits, Rm/Rn unsigned)
				{Rd, Ra} <= Rn & Rm;
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
module memoryInst(
	cond,
	Op,
	Funct,
	Rn,
	Rd,
	Src2,
	Mem,
	Adr
)
	input wire cond[3:0];
	input wire Op[1:0];
	input wire Funct[6:0]
	input wire [31:0] Adr;
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
				if (Funct[2] == 1'b1) begin
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
				if (op2 == 2'b10) begin
					if (L == 1'b1) begin
						// Rd ← Mem[Adr]7:0
						Rd <= Mem[7:0];
					end
				end
				if (op2 == 2'b11) begin
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

module regInstBranch(
	cond,
	Op,
	funct,
	immd,
);
	input reg cond[3:0];
	input reg Op[1:0];
	input reg funct[1:0];
	input reg immd[23:0];
	reg PC[31:0]; 
	reg LR[31:0];

	always @(*) begin
		if (funct == 2'b00) begin
			// PC ← (PC+8)+imm24 << 2
			PC
		end
		if (funct != 2'b01) begin
			// LR ← (PC+8) – 4; PC ← (PC+8)+imm24 << 2
		end
	end


endmodule

// register file for data processing
module regDataProcessing(
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
				else if (I == 1'b0) begin
					case (Src2[6:5])
						2'b00: 
							if (Src2[11:4] != 8'b0) begin
								//Rd ← Rm << Src2
							end
						2'b01: //Rd ← Rm >> Src2
						2'b10: //Rd ← Rm>>>Src2
						2'b11:
							case (Src2[11:7])
								4'b0: //{Rd, C} ← {C, Rd}
								default: //Rd ← Rn ror Src2
							endcase
						default: 
					endcase
				end
			4'b1110: Rd <= Rn & Src2;
			4'b1111: Rd <= Rn & Src2;
			default:
		endcase
	end

endmodule