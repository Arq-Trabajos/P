    ; Implementación de multiplicadores

	; MUL recibe dos valores de 32 bits y devuelve valores de 32 bits
	MOV R1, #4          ; R1 = 4
	MUL R2, R1, R1     ; R2 = R1*R1


	; UMULL recibe valores enteros de 32 bits sin signo y devuelve valores de 64 bits con valores de entrada de 64 bits.
    MOV R1, #0x00101          ; R1 = 5
	UMULL R2,R3, R1, R1        ; {R2,R3} = R1*R1 : R2 almacena el valor de un word y R3 almacena el valor de otro word, de esa manera se forma el 64 bits.
    
	; SMULL recibe valores de 32 bits con signo interpretados como complemento de 2 y retorna valores de 64 bits
    MOV R1, #0x0111           ; R1 = -1 : 001 -> 110 -> 110 + 1 -> 111
    SMULL R2, R3, R1, R1      ; {R2,R3} = R1*R1  : R2 almacena el valor de un word y R3 almacena el valor de otro word, de esa manera se forma el 64 bits.

