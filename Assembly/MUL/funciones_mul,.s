; Recibe entradas de 32 bits y retorna salida de 32 bits
; POTENCIATEST ASIGNA UN VALOR N Y GUARDA LA POTENCIA DEL NÚMERO EN #0X001

POTENCIATEST
    MOV R4, #4
    BL POTENCIA
    MOV R1, [#0x001]
    STR R0, R1

POTENCIA
    PUSH {R4,LR}   ; guardamos R4 y R0 en el stack, siendo R4 el parámetro y R0 el valor de retorno
    MUL R0, R4,R4   ; R0 = R4*R4
    POP {R4, LR}    ; return R0
    MOV PC, LR




