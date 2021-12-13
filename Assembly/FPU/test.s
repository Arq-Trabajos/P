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

// FSMUL y FHMUL Toman el supuesto lugar de TST y TEQ
// 00000011000100100001101010100001 | | PARA EL FSMUL
// 00000011001100100001101011100001 | | PARA EL FHMUL
//Guardar STR
// 11100101100001000010000001100100 E5842064
// 11100101100001000011000001101000 E5843068