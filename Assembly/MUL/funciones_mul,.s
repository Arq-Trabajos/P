; Recibe entradas de 32 bits y retorna salida de 32 bits
; POTENCIATEST ASIGNA UN VALOR N Y GUARDA LA POTENCIA DEL NÚMERO EN #0X001

POTENCIATEST:
    MOV R4, #4
    BL POTENCIA
;RESULT
POTENCIA:
    PUSH {R4,LR}   
    MUL R0, R4,R4   
    POP {R4, LR}   
    MOV PC, LR



.global _start
_start:
	
	
POTENCIATEST:
    MOV R4, #4
    BL POTENCIA

POTENCIA:
    PUSH {R4,LR}   ; guardamos R4 y R0 en el stack, siendo R4 el parámetro y R0 el valor de retorno
    MUL R0, R4,R4  ; R0 = R4*R4
    POP {R4, LR}
    MOV PC, LR     ; return R0


