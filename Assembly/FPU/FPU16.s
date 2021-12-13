; R4, R5 son mantizas de a y b. R6 y R7 son exponentes de a y b
FLPADD
  PUSH {R4, R5, R6, R7, R8}         ; colocamos en el stack los registros que utilizaremos en la funcion
  LDR R2, #0x007FFFFF               ; cargamos la máscara de la mantisa
  LDR R3, #0X7F800000               ; cargamos la máscara de exponente
  AND R4, R0, R2                    ; extraemos la mantisa del R0
  AND R5, R1, R2                    ; extraemos la mantisa del R1
  ORR R4, R4, #0X80000000           ; insertamos ...
  ORR R5, R5, #0X80000000           ; insertamos ...
  AND R6, R0, R3                    ; extraemos el exponente del R0
  LSR R6, R6, #23                   ; realizamos shift al exponente hacia la derecha
  AND R7, R1, R3                    ; extraemos el exponente del R1
  LSR R7, R7, #23                   ; realizamos shift al exponente hacia la derecha

MATCH
  CMP R6,R7
  BEQ ADDMANTISA
  BHI SHIFTB

SHIFTA
  SUB R8, R7, R6
  ASR R4, R4, R8
  ADD R6,R6, R8
  B  ADDMANTISA

SHIFTB
  SUB R8, R6, R7
  ASR R5, R5, R8

ADDMANTISA
  ADD R4, R4, R5

NORMALIZE
  ANDS R5, R4, #0x10000000
  BEG DONE
  LSR R4, R4, #1
  ADD R6, R6, #1

DONE
  AND R4, R4, R2
  LSL R6, R6, #23
  ORR R0, R4, R6
  POP {R4,R5,R6,R7,R8}
  MOV PC, LR