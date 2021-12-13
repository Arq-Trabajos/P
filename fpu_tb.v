`include "FPU.v"
module fpu_tb;

reg fpucontrol, floatType;
reg [31:0] A, B;
wire [3:0] fpuFlags;
wire [31:0] Result;

FPU test(fpucontrol, A, B, floatType,  fpuFlags, Result);

initial begin

    $display("HALF PRECISION:");
    //16 bits test
    floatType = 1'b0;

    //Adicion
    B = 32'h00003555; //0.33325195
    A = 32'h00003bff; //0.99951172
    fpucontrol = 1'b0;

    #1;
    $display("%x + %x = %x; FLAGS: %B", A, B, Result, fpuFlags);

    //multiplicacion
    B = 32'h00005664; //102.25
    A = 32'h0000d482; //-72.1
    fpucontrol = 1'b1;
    
    #1;
    $display("%x x %x = %x; FLAGS: %B", A, B, Result, fpuFlags);

    // 32 bits test
    $display("SINGLE PRECISION:");
    floatType = 1'b1;

    //Adicion
    B = 32'h418c0000; //17.5
    A = 32'hc059999a; //89.234
    fpucontrol = 1'b0;

    #1;
    $display("%x + %x = %x; FLAGS: %B", A, B, Result, fpuFlags);
    //Multiplicacion 
    B = 32'h418c0000; //17.5
    A = 32'hc059999a; //-3.4
    fpucontrol = 1'b1;
    
    #1;
    $display("%x x %x = %x; FLAGS: %B", A, B, Result,  fpuFlags);
end

endmodule