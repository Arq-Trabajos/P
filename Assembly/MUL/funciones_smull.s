; Ahora implementaremos una función para SMULL
; En este caso implementaremos una función que retorne la multiplicación de dos valores

MULTIPLICACIONSIGNOTEST:
    MOV R1, #0x110
    MOV R2, #0x010
    Bl MULTIPLICACIONSIGNO

;Result R4, R5 -> el número final es [R5, R4] en 64 bits

MULTIPLICACIONSIGNO:
    PUSH {R1,R2, LR}
    SMULL {R4,R5} R1, R2
    POP {R1, R2, LR}    
    MOV PC, LR