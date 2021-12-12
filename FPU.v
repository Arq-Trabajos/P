module fpu_tb;

reg fpucontrol;
reg [31:0] A, B;
wire [3:0] ALUFlags;
wire [31:0] Result;

FPU test(fpucontrol, A, B,  ALUFlags, Result);

initial begin
    A = 32'h477FFFB0;
    
end

endmodule

module FPU(
    input FPUControl, input [31:0] A, B,
    output [3:0] ALUFlags, output reg [31:0] Result
);

    
    // Filtrar exp1
    reg [31:0] expA, expB;
    reg [31:0] manA, manB;
    expA = A[30:23];
    manA = 32'h800000 + A[22:0]; 
    expB = B[30:23];
    manB = 32'h800000 + B[22:0]; 

    wire [31:0] shift;
    if(expA > expB) begin
        assign shift = expA - expB;
    end else begin
        assign shift = expB - expA;
    end

    reg [31:0] expFinal;
    if(A>B) begin
        manB = manB>>shift;
        expFinal = expA;
    end else begin
        manA = manA>>shift;
        expFinal = expB;
    end

    reg [31:0] manSum;
    manSum = manA + manB;
    
    if (manSum > 32'hFFFFFF) begin
        manSum = manSum1>>1;
        expFinal = expFinal + 1;
    end

    
    

endmodule