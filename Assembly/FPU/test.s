//Lo que se quiere hacer es crear una función que eleve al cuadrado un numero ya sea para 16 y 32 bits

MAIN: //Definicion de valores
LDR R0, =0x5664 //102.25 EN 16 BITS
LDR R1, =0x418c0000 //17.5 en 32 bits

QUADRATIC32:
    FSMUL R2, R1, R1 //Floating-SinglePrecision-Multiplication

QUADRATIC16:
    FHMUL R3, R0, R0 //Floating-HalfPrecision-Multiplication

DONE:
    // SE guardan los valores
    STR R2, [R4, #100]
    STR R3, [R4, #104]