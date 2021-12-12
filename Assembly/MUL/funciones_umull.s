; UMULL recibe valores enteros de 32 bits sin signo y devuelve valores de 64 bits con valores de entrada de 64 bits.
; el máximo valor de 32 bits a considerar es (2^32) - 1

; Recordar que al multiplicar una serie de bits de tamaño n con otro siendo n la cantidad máxima de bits entre los dos operandos
; el máximo valor que puede retornar es 2n

INCREMENTMODULETEST
    MOV R2, #2              ; R2 = 2
    MOV R3, #3              ; R3 = 3 -> v = (R2, R3) vector
    B INCREMENTMODULO
; R1 y R2 result
INCREMENTMODULO             ; v = (x,y) , 2v = (2x, 2y)
    PUSH {R2, R3, LR}       ; siendo R2 el vector posicion, R3 el incremento y R0 el parámetro de retorno
    UMULL R4, R5, R2, R3    ; Siendo R4 LSB y R5 MSB, se obtiene {R4, R5} = R2*R3
    MOV R0, R4
    MOV R1, R5
    POP {R2, R3, LR}
    MOV PC, LR
    
