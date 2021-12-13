module FPU(
    input FPUControl, input [31:0] A, B, input floatType,
    output [3:0] FPUFlags, output reg [31:0] Result
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
                //FILTRAMOS MANTISAS Y EXPONENTES
                mantA = {1'b1,A[9:0]};
                mantB = {1'b1,B[9:0]};
                expA = A[14:10];
                expB = B[14:10];

                //ALGORITMO DE ADICION
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

                // ALGORITMO DE MULTIPLICACION
                if(A==0 | B==0) begin
                    Prod = 32'b0;
                end else begin
                    expFinPROD = expA+expB-15;
                    mantFinPROD = mantA*mantB;
                end

                for(i = 63; ~mantFinPROD[63]; i=i-1)begin
                    mantFinPROD = mantFinPROD<<1;
                end


                Prod = {A[15]^B[15], expFinPROD[4:0], mantFinPROD[62:53]};

            end
            1'b1: begin
                // FILTROS
                mantA = {1'b1,A[22:0]};
                mantB = {1'b1,B[22:0]};
                expA = A[30:23];
                expB = B[30:23];

                // ALGORITMO DE ADICION
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

                // ALGORITMO DE MULTIPLICACION
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


        Result = ~FPUControl?Sum:Prod; // dEPENDIENDO DE LAS DOS OPERACIONES, LA SALIDA DEL FPU TENDRÁ LA OPERACIÓN DE SUMA O PRODUCTO
    end

    wire Negative, Overflow, Carry, Zero; 
    assign Zero = floatType?32'b0==Result: 16'b0==Result[15:0]; // 000...000 DEFINE EL NUMERO 0 
    assign Negative = floatType?Result[31]:Result[15]; // LA NEGATIVIDAD SE DEFINE POR EL PRIMER BIT
    assign Overflow = floatType?Result==32'hffffffff|Result==32'h7fffffff:Result[15:0]==16'hffff|Result[15:0]==316'h7fff; 
    // LOS ÚNICOS VALORES QUE LA NOTACIÓN DE COMA FLOTANTE NO RECONOCE YA QUE SOBREPASAN SU CANTIDAD DE VALORES SON: 1111...1111 Y 01111.1111
    assign Carry = 0;
    // EN LA NOTACIÓN COMA FLOTANTE NO SE PRESENTA CARR

    assign FPUFlags = {Negative, Zero, Carry, Overflow};

endmodule