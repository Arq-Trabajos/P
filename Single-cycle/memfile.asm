// Example: Test ARM processor

// MAIN	 SUB R0, R15, R15 	; R0 = 0				
	1110 000 0010 0 1111 0000 0000 0000 1111 E04F000F 0x00
//  		 ADD R2, R0, #5      	; R2 = 5             
	1110 001 0100 0 0000 0010 0000 0000 0101 E2802005 0x04
//  		 ADD R3, R0, #12    	; R3 = 12            
	1110 001 0100 0 0000 0011 0000 0000 1100 E280300C 0x08

