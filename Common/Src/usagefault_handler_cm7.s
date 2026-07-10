; Save fault-time r4-r11 before any C prologue, then dispatch to C handler.
                PRESERVE8
                THUMB
                REQUIRE8

                AREA    |.text|, CODE, READONLY
                EXPORT  UsageFault_Handler
                IMPORT  UsageFault_Handler_C

UsageFault_Handler PROC
                TST     LR, #4
                ITE     EQ
                MRSEQ   R0, MSP
                MRSNE   R0, PSP
                PUSH    {R4-R11}
                MOV     R1, SP
                ADD     R0, R1, #32
                BL      UsageFault_Handler_C
                POP     {R4-R11}
                BX      LR
                ENDP

                END
