`include "ALU.v"
module top ( // instanciando todas las variables de memory y el arm
    clk,
    reset,
    WriteData,
    Adr,
    MemWrite,
);
    input wire clk;
    input wire reset;
    output wire [31:0] WriteData;
    output wire [31:0] Adr;
    output wire MemWrite;
    wire [31:0] ReadData;

    arm arm(
        .clk(clk),
        .reset(reset),
        .MemWrite(MemWrite),
        .Adr(Adr),
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

module arm (
    clk,
    reset,
    MemWrite,
    Adr,
    WriteData,
    ReadData
);
    input wire clk;
    input wire reset;
    output wire MemWrite;
    output wire [31:0] Adr;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;
    wire [31:0] Instr;
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
    
    wire SrcExtra;
    wire FPUWrite;
    wire RegSrcExtra;

    controller c( // module de control unit dentro de nuestro procesador
        .clk(clk),
        .reset(reset),
        .Instr(Instr),
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
        .ALUControl(ALUControl),
        .SrcExtra(SrcExtra),
        .FPUWrite(FPUWrite),
        .RegSrcExtra(RegSrcExtra)
    );

    datapath dp( // module de datapath de nuestro procesador, esto subsecuente mente se divide a mas submodulos de filtro de infromacion
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
        .ALUControl(ALUControl),
        .SrcExtra(SrcExtra),
        .FPUWrite(FPUWrite),
        .RegSrcExtra(RegSrcExtra)
    );
endmodule

module condcheck ( // este modulo se puede llamar para prender los flags de negative zero, carry  y overflow estan prendidas
    Cond,
    Flags,
    CondEx
);
    input wire [3:0] Cond; // cond nos permite saber cual de los flags se va a prender y pasamos a condex los resultados
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
    FpuW,
    PCWrite,
    RegWrite,
    MemWrite,
    FPUWrite
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
    input wire FpuW;

    output wire PCWrite;
    output wire RegWrite;
    output wire MemWrite;
    output wire FPUWrite;

    wire [1:0] FlagWrite;
    wire [3:0] Flags;
    wire CondEx;
    wire CondExFlop;
    condcheck cc(
        .Cond(Cond),
        .Flags(Flags),
        .CondEx(CondEx)
    );

    flopr #(1) condExReg(
        .clk(clk),
        .reset(reset),
        .d(CondEx),
        .q(CondExFlop)
    );

    flopenr #(2) ALUF1Reg(
        .clk(clk),
        .reset(reset),
        .en(FlagWrite[1]),
        .d(ALUFlags[3:2]),
        .q(Flags[3:2])
    );

    flopenr #(2) ALUF2Reg(
        .clk(clk),
        .reset(reset),
        .en(FlagWrite[0]),
        .d(ALUFlags[1:0]),
        .q(Flags[1:0])
    );

    assign FlagWrite = FlagW & {2 {CondEx}};
    assign MemWrite = CondExFlop & MemW;
    assign RegWrite = CondExFlop & RegW;
    assign FPUWrite = CondExFlop & FpuW;
    assign PCWrite = NextPC | (PCS & CondExFlop);

endmodule

module controller ( // se va a encargar de las instrucciones por medio del decoder mientras revisa el condition logic
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
    ALUControl,
    SrcExtra,
    FPUWrite,
    RegSrcExtra
);
    input wire clk;
    input wire reset;
    input wire [31:0] Instr;
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
    output wire SrcExtra;
    output wire FPUWrite;
    output wire RegSrcExtra;

    wire [1:0] FlagW;
    wire PCS;
    wire NextPC;
    wire RegW;
    wire MemW;
    wire FpuW;

    decode dec(
        .clk(clk),
        .reset(reset),
        .Op(Instr[27:26]),
        .Mop(Instr[7:4]),
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
        .ALUControl(ALUControl),
        .SrcExtra(SrcExtra),
        .FpuW(FpuW),
        .RegSrcExtra(RegSrcExtra)
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
        .FpuW(FpuW),
        .PCWrite(PCWrite),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite),
        .FPUWrite(FPUWrite)
    );
endmodule

module datapath ( // datapath abre a todo submodulo usado en el resto del programa
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
    ALUControl,
    SrcExtra,
    FPUWrite,
    RegSrcExtra
);
    input wire clk;
    input wire reset;
    output wire [31:0] Adr;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;
    output wire [31:0] Instr;
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
    
    input wire SrcExtra;
    input wire FPUWrite;
    input wire RegSrcExtra;

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
    wire [31:0] ALUResultEstra;
    wire [31:0] ALUOut;
    wire [31:0] ALUOut2;
    wire [3:0] RA1;
    wire [3:0] RA2;
    wire [63:0] FRD1;
    wire [63:0] FRD2;
    wire [63:0] FA;
    wire [63:0] FWriteData;
    wire [63:0] FResult;
    wire [63:0] FPUResult;

    flopenr #(32) pcreg(
        .clk(clk),
        .reset(reset),
        .en(PCWrite),
        .d(PCNext),
        .q(PC)
    );

    mux2 #(32) adrmux(
        .d0(PC),
        .d1(PCNext),
        .s(AdrSrc),
        .y(Adr)
    );

    flopenr #(32) instrreg(
        .clk(clk),
        .reset(reset),
        .en(IRWrite),
        .d(ReadData),
        .q(Instr)
    );

    flopr #(32) datareg(
        .clk(clk),
        .reset(reset),
        .d(ReadData),
        .q(Data)
    );

    wire [3:0] _RA1;
    mux2 #(4) ra1mulmux(
        .d0(Instr[19:16]),
        .d1(Instr[3:0]),
        .s(RegSrcExtra),
        .y(_RA1)
    );

    mux2 #(4) ra1mux(
        .d0(_RA1),
        .d1(4'd15),
        .s(RegSrc[0]),
        .y(RA1)
    );

    wire [3:0] _RA2;
    mux2 #(4) ra2mulmux(
        .d0(Instr[3:0]),
        .d1(Instr[11:8]),
        .s(RegSrcExtra),
        .y(_RA2)
    );

    mux2 #(4) ra2mux(
        .d0(_RA2),
        .d1(Instr[15:12]),
        .s(RegSrc[1]),
        .y(RA2)
    );

    wire [3:0] A3;
    mux2 #(4) a3mux(
        .d0(Instr[15:12]),
        .d1(Instr[19:16]),
        .s(RegSrcExtra),
        .y(A3)
    );


    regfile rf(
        .clk(clk),
        .WriteEn3(RegWrite),
        .ra1(RA1),
        .ra2(RA2),
        .WriteA3(A3),
        .WriteA3Extra(Instr[15:12]),
        .WriteD3(Result),
        .WriteD3Extra(ALUOut2),
        .SrcExtra(lmulFlag),
        .r15(Result),
        .rd1(RD1),
        .rd2(RD2)
    );

    extend ext(
        .Instr(Instr[23:0]),
        .ImmSrc(ImmSrc),
        .ExtImm(ExtImm)
    );

    flopr #(64) rdreg(
        .clk(clk),
        .reset(reset),
        .d({RD1, RD2}),
        .q({A, WriteData})
    );

    mux2 #(32) srcamux(
        .d0(A),
        .d1(PC),
        .s(ALUSrcA[0]),
        .y(SrcA)
    );

    mux3 #(32) srcbmux(
        .d0(WriteData),
        .d1(ExtImm),
        .d2(32'd4),
        .s(ALUSrcB),
        .y(SrcB)
    );

    alu a(
        .A(SrcA),
        .B(SrcB),
        .ALUControl(ALUControl),
        .Result(ALUResult),
        .ResultExtra(ALUResultEstra),
        .ALUFlags(ALUFlags)
    );

    fpu_regfile fpu_regfile(
        .clk(clk),
        .WriteEn3(FPUWrite),
        .ra1(Instr[19:16]),
        .ra2(Instr[3:0]),
        .WriteA3(Instr[15:12]),
        .A1(Instr[7]),
        .A2(Instr[5]),
        .A3(Instr[6]),
        .instr8(Instr[8]),
        .WriteD3(FResult),
        .rd1(FRD1),
        .rd2(FRD2)
    );

    flopr #(128) frdreg(
        .clk(clk),
        .reset(reset),
        .d({FRD1, FRD2}),
        .q({FA, FWriteData})
    );

    fpu f(
        .a(FA),
        .b(FWriteData),
        .double(Instr[8]),
        .Result(FPUResult)
    );

    flopr #(64) fpureg(
        .clk(clk),
        .reset(reset),
        .d(FPUResult),
        .q(FResult)
    );

    flopr #(32) alureg(
        .clk(clk),
        .reset(reset),
        .d(ALUResult),
        .q(ALUOut)
    );

    flopr #(32) alureg2(
        .clk(clk),
        .reset(reset),
        .d(ALUResultEstra),
        .q(ALUOut2)
    );

    mux3 #(32) resultmux(
        .d0(ALUOut),
        .d1(Data),
        .d2(ALUResult),
        .s(ResultSrc),
        .y(Result)
    );

    assign PCNext = Result;

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
module decode (
    clk,
    reset,
    Op,
    Mop,
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
    ALUControl,
    SrcExtra,
    FpuW,
    RegSrcExtra
);
    input wire clk;
    input wire reset;
    input wire [1:0] Op;
    input wire [5:0] Funct;
    input wire [3:0] Mop;
    input wire [3:0] Rd;
    output reg [1:0] FlagW;
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
    output wire SrcExtra;
    output reg [2:0] ALUControl;
    output wire FpuW;
    output wire RegSrcExtra;


    wire Branch;
    wire ALUOp;
    reg FlagExtrab;

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
        .ALUOp(ALUOp),
        .FlagExtrab(FlagExtrab),
        .SrcExtra(SrcExtra),
        .FpuW(FpuW)
    );

    always @(*) begin
        FlagExtrab = 0;
        if (ALUOp) begin
            if(Mop[3:0] == 4'b1001) begin
                case(Funct[4:1])
                    4'b0000: ALUControl = 3'b101; // Añadimos para el MUL
                    4'b0100: begin
                         FlagExtrab = 1;
                         ALUControl = 3'b110; // Añadimos para el UMULL
                    end
                    4'b0110: begin
                         FlagExtrab = 1;
                         ALUControl = 3'b111; //Añadimos para el SMULL
                    end
                endcase
            end
            else begin
                case (Funct[4:1])
                    4'b0100: ALUControl = 3'b000; // Añadimos para el ADD
                    4'b0010: ALUControl = 3'b001; // Añadimos para el SUB
                    4'b0000: ALUControl = 3'b010; // Añadimos para el AND
                    4'b1100: ALUControl = 3'b011; // Añadimos para el ORR

                    4'b0001: ALUControl = 3'b100; // Añadimos para el EOR

                    default: ALUControl = 3'bxx;
                endcase
            end
            FlagW[1] = Funct[0];
            FlagW[0] = Funct[0] & ((ALUControl == 3'b000) | (ALUControl == 3'b001));
        end
        else begin
            ALUControl = 3'b000;
            FlagW = 2'b00;
        end
    end

    // PC Logic

    assign PCS = ((Rd == 4'b1111) & RegW) | Branch;

    // Instr Decoder
    assign ImmSrc = Op;
    assign RegSrc[0] = (Op == 2'b10);
    assign RegSrc[1] = (Op == 2'b01);

    assign RegSrcExtra = Mop[3:0] == 4'b1001;

endmodule

module fpu_single_adder (R1, R2, result);
    input [31:0] R1, R2;
    output [31:0] result;

    wire [31:0] exponente1, exponente2;

    assign exponente1 = {24'b0, R1[30:23]}; assign exponente2 = {24'b0, R2[30:23]};

    //extract the mantissa (23-bits) of 2 input and add 1 in the mantissa
    wire [31:0] mantisa1;
    wire [31:0] mantisa2;

    assign mantisa1 = {8'b0, 1'b1, R1[22:0]}; assign mantisa2 = {8'b0, 1'b1, R2[22:0]};

    //compare exponent
    wire [31:0] diferencia;
    assign diferencia = exponente1 - exponente2;

    //save the higher exponent in exponent result
    wire [31:0] exponenteResult;
    assign exponenteResult = (diferencia > 0) ? exponente1 : exponente2;

    wire [31:0] mantisaMenor, mantisaMayor;

    //assign mantissas depende of the result of dif
    assign {mantisaMenor, mantisaMayor} = (diferencia > 0) ? {mantisa2, mantisa1} : {mantisa1, mantisa2};

    //shifting the result of dif in the min mantissa 
    wire [31:0] mantisaWithShift = mantisaMenor >> diferencia;

    //add both mantissas
    wire [31:0] sum;
    assign sum = mantisaMayor + mantisaWithShift;

    //normalize the mantissa
    wire [31:0] sum2;
    assign sum2 = (sum[31:24] != 0) ? sum >> 1 : sum;

    //adjust the exponent
    wire [31:0] exponenteWithshift;
    assign exponenteWithshift = sum[31:24] != 0 ? exponenteResult + 1 : exponenteResult;

    //concatenate the sign bit, exponent and mantissa
    assign result = {1'b0, exponenteWithshift[7:0], sum2[22:0]};

endmodule


module fpu(a, b, double, Result);
    input [63:0] a, b;
    input double;

    output [63:0] Result;

    wire [31:0] f_result;
    wire [63:0] d_result;

    fpu_single_adder float_a(
        .R1(a[31:0]),
        .R2(b[31:0]),
        .result(f_result)
    );

    double_adder double_a(
        .srcA(a),
        .srcB(b),
        .result(d_result)
    );

    assign Result = double ? d_result : {32'h0000, f_result} ;

endmodule

module fpu_regfile (
    clk,
    WriteEn3,
    ra1,
    ra2,
    WriteA3,
    A1,
    A2,
    A3,
    instr8,
    WriteD3,
    Rd,
    rd2
);
    input wire clk;
    input wire WriteEn3;

    input wire [3:0] ra1;
    input wire [3:0] ra2;
    input wire [3:0] WriteA3;

    input wire A1;
    input wire A2;
    input wire A3;

    input wire instr8;

    input wire [63:0] WriteD3;

    output wire [63:0] Rd;
    output wire [63:0] rd2;

    reg [63:0] rf [15:0];

    always @(posedge clk)
        if (WriteEn3)
            if(instr8 == 1)
                rf[WriteA3] <= WriteD3;
            else // Single
                if(A3 == 1)
                    rf[WriteA3][63:32] <= WriteD3[31:0];
                else
                    rf[WriteA3][31:0] <= WriteD3[31:0];

    wire [31:0] Rdf = A1 == 1 ? rf[ra1][63:32] : rf[ra1][31:0];
    wire [31:0] rd2f = A2 == 1 ? rf[ra2][63:32] : rf[ra2][31:0];

    assign Rd = instr8 == 1 ? rf[ra1] : {32'b0, Rdf};
    assign rd2 = instr8 == 1 ? rf[ra2] : {32'b0, rd2f};
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
    ALUOp,
    FlagExtrab,
    SrcExtra,
    FpuW
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
    output wire SrcExtra;
    output wire FpuW;
    reg [3:0] state;
    reg [3:0] nextstate;
    reg [14:0] controls;
    input wire FlagExtrab;

    localparam [3:0] FETCH    = 0;
    localparam [3:0] DECODE   = 1;
    localparam [3:0] MEMADR   = 2;
    localparam [3:0] MEMRD    = 3;
    localparam [3:0] MEMWB    = 4;
    localparam [3:0] MEMWR    = 5;
    localparam [3:0] EXECUTER = 6;
    localparam [3:0] EXECUTEI = 7;
    localparam [3:0] ALUWB    = 8;
    localparam [3:0] BRANCH   = 9;
    localparam [3:0] UNKNOWN  = 10;
    localparam [3:0] EXECUTEF = 11; 
    localparam [3:0] FPUWB    = 12; 
    localparam [3:0] ALUWB64   = 13; 
    // state register
    always @(posedge clk or posedge reset)
        if (reset)
            state <= FETCH;
        else
            state <= nextstate;

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
                    2'b11: nextstate = EXECUTEF;
                    default: nextstate = UNKNOWN;
                endcase
            MEMADR:
                if(Funct[0])
                    nextstate = MEMRD;
                else
                    nextstate = MEMWR;
            MEMRD:    nextstate = MEMWB;
            MEMWB:    nextstate = FETCH;
            MEMWR:    nextstate = FETCH;
            EXECUTER: 
                begin
					if(FlagExtrab == 1'b1) begin
						nextstate = ALUWB64;
					end
					else begin
						nextstate = ALUWB;
					end
				end
            EXECUTEI: 
                begin
					if(FlagExtrab == 1'b1) begin
						nextstate = ALUWB64;
					end
					else begin
						nextstate = ALUWB;
					end
				end
            EXECUTEF: nextstate = FPUWB;
            FPUWB:    nextstate = FETCH;
            ALUWB:    nextstate = FETCH;
            ALUWB64:   nextstate = FETCH;
            BRANCH:   nextstate = FETCH;
            default:  nextstate = FETCH;
        endcase

    // state-dependent output logic
    always @(*)
        case (state)
            FETCH:    controls = 15'b001000101001100;
            DECODE:   controls = 15'b000000001001100;
            EXECUTER: controls = 15'b000000000000001;
            EXECUTEI: controls = 15'b000000000000011;
            ALUWB:    controls = 15'b000001000000000;
            MEMADR:   controls = 15'b000000000000010;
            MEMWR:    controls = 15'b000010010000000;
            MEMRD:    controls = 15'b000000010000000;
            MEMWB:    controls = 15'b000001000100000;
            BRANCH:   controls = 15'b000100001000010;
            EXECUTEF: controls = 15'b000000000000000;
            FPUWB:    controls = 15'b010000000000000;
            ALUWB64:   controls = 15'b100001000000000;
            default:  controls = 15'bxxxxxxxxxxxxxxx;
        endcase
    assign {SrcExtra, FpuW, NextPC, Branch, MemW, RegW, IRWrite, AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, ALUOp} = controls;
endmodule

module regfile (
    clk,
    WriteEn3,
    ra1,
    ra2,
    WriteA3,   
    WriteA3Extra,
    WriteD3
    WriteD3Extra,
    SrcExtra,    
    r15,
    rd1,
    rd2
);
    input wire clk;
    input wire WriteEn3;
    input wire [3:0] ra1;
    input wire [3:0] ra2;
    input wire [3:0] WriteA3;
    input wire [3:0] WriteA3Extra;
    input wire [31:0] WriteD3;
    input wire [31:0] WriteD3Extra;
    input wire SrcExtra;
    input wire [31:0] r15;
    output wire [31:0] rd1;
    output wire [31:0] rd2;
    reg [31:0] rf [14:0];

    always @(posedge clk)
        if (WriteEn3) begin
            rf[WriteA3] <= WriteD3;
            if(SrcExtra) begin
                rf[WriteA3Extra] <= WriteD3Extra;
            end
        end

    assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
    assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule

module double_adder (srcA, srcB, result);
    input [63:0] srcA;
    input [63:0] srcB;

    output [63:0] result;

    wire [63:0] expA = {56'b0, srcA[62:52]};
    wire [63:0] expB = {56'b0, srcB[62:52]};

    wire [63:0] manA = {11'b0, 1'b1, srcA[51:0]};
    wire [63:0] manB = {11'b0, 1'b1, srcB[51:0]};

    wire [63:0] diff = expA - expB;
    wire [63:0] expR = diff > 0 ? expA : expB;

    wire [63:0] manMin;
    wire [63:0] manMax;

    assign {manMin, manMax} = diff > 0 ? {manB, manA} : {manA, manB};

    wire [63:0] manMinShift = manMin >> diff;

    wire [63:0] sum = manMax + manMinShift;
    wire [63:0] _sum = sum[63:53] != 0 ? sum >> 1 : sum;
    wire [63:0] expRshift = sum[63:53] != 0 ? expR + 1 : expR;

    assign result = {1'b0, expRshift[10:0], _sum[51:0]};

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
    reg [31:0] RAM [0:127];
    initial $readmemh("memfile.dat", RAM);
    assign rd = RAM[a[31:2]];
    always @(posedge clk)
        if (we)
            RAM[a[31:2]] <= wd;
endmodule