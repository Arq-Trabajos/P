module fpu_tb;

reg fpucontrol, floatType;
reg [31:0] A, B;
wire [3:0] fpuFlags;
wire [31:0] Result;

FPU test(fpucontrol, A, B, floatType,  fpuFlags, Result);

initial begin

    //16 bits test
    floatType = 1'b0;

    //Adicion
    B = 32'h00003555; //0.33325195
    A = 32'h00003bff; //0.99951172
    fpucontrol = 0;

    #1;
    $display("%x + %x = %x", A, B, Result);

    //multiplicacion
    B = 32'h00005664; //102.25
    A = 32'h0000d482; //-72.1
    fpucontrol = 1;
    
    #1;
    $display("%x x %x = %x", A, B, Result);

    // 32 bits test
    floatType = 1'b1;

    //Adicion
    B = 32'h418c0000; //17.5
    A = 32'hc059999a; //89.234
    fpucontrol = 0;

    #1;
    $display("%x + %x = %x", A, B, Result);
    //Multiplicacion 
    B = 32'h418c0000; //17.5
    A = 32'hc059999a; //-3.4
    fpucontrol = 1;
    
    #1;
    $display("%x x %x = %x", A, B, Result);
end

endmodule

module FPU(
    input FPUControl, input [31:0] A, B, input floatType,
    output [3:0] ALUFlags, output reg [31:0] Result
);

    // FPUCONtrol: ADD O MUL
    // FloatTYPE: 16 o 32

    reg [31:0] mantA, mantB;
    reg [31:0] expA, expB;
    //ADD
    reg [31:0] mantFinADD, expFinADD;
    reg [31:0] expDif;
    reg [31:0] norm;

    //Producs
    reg[63:0] mantFinPROD, expFinPROD;

    integer i;


    reg[31:0] Sum;
    reg[31:0] Prod;

    always @(*) begin

    // 0: Half
    // 1: Single
        case(floatType)
            1'b0: begin
                mantA = {1'b1,A[9:0]};
                mantB = {1'b1,B[9:0]};
                expA = A[14:10];
                expB = B[14:10];

                //ADICION
                if(expA>expB) begin
                    expDif = expA-expB;
                    expFinADD = expA;
                end else begin
                    expDif = expB-expA;
                    expFinADD = expB;
                end

                if(A>B)begin
                    mantB = mantB >> expDif;
                end else begin
                    mantA = mantA >> expDif;
                end

                mantFinADD = mantA + mantB;
                norm = 32'h00000400;

                if(norm<mantFinADD) begin
                    mantFinADD = mantFinADD >> 1;
                    expFinADD = expFinADD + 1;
                end

                Sum = {expFinADD[4:0], mantFinADD[9:0]};

                // PRODUCTOS
                if(A==0 | B==0) begin
                    Prod = 32'b0;
                end else begin
                    expFinPROD = expA+expB-15;
                    mantFinPROD = mantA*mantB;
                end

                for(i = 63; ~mantFinPROD[63]; i=i-1)begin
                    mantFinPROD = mantFinPROD<<1;
                end


                Prod = {A[31]^B[31], expFinPROD[4:0], mantFinPROD[62:53]};

            end
            1'b1: begin
                // FILTROS
                mantA = {1'b1,A[22:0]};
                mantB = {1'b1,B[22:0]};
                expA = A[30:23];
                expB = B[30:23];

                // ADICION
                if(expA>expB) begin
                    expDif = expA-expB;
                    expFinADD = expA;
                end else begin
                    expDif = expB-expA;
                    expFinADD = expB;
                end

                if(A>B)begin
                    mantB = mantB >> expDif;
                end else begin
                    mantA = mantA >> expDif;
                end

                mantFinADD = mantA + mantB;
                norm = 32'h00800000;

                if(norm<mantFinADD) begin
                    mantFinADD = mantFinADD >> 1;
                    expFinADD = expFinADD + 1;
                end

                Sum = {expFinADD[7:0], mantFinADD[22:0]};

                // PRODUCTOS
                if(A==0 | B==0) begin
                    Prod = 32'b0;
                end else begin
                    expFinPROD = expA+expB-127;
                    mantFinPROD = mantA*mantB;
                end

                for(i = 63; ~mantFinPROD[63]; i=i-1)begin
                    mantFinPROD = mantFinPROD<<1;
                end


                Prod = {A[31]^B[31], expFinPROD[7:0], mantFinPROD[62:40]};
            end
        endcase


        Result = ~FPUControl?Sum:Prod;
    end

    

endmodule