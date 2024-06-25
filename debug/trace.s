.syntax unified
.thumb
.text

/* NOTE: If use this file's HardFault_Handler, please comments the HardFault_Handler code on other file. */

.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
    MOV     r0, lr                  /* get lr */
    MOV     r1, sp                  /* get stack pointer (current is MSP) */
    BL      trace_analysiser

Fault_Loop:
    BL      Fault_Loop              /* while(1) */
