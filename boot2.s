.cpu cortex-m0plus
.thumb

.section .boot2, "ax"
    ldr r0, =XIP_SSI_SSIENR
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =XIP_SSI_CTRLR0 ;@   2832 2427 2023 1619 12.15 8.11 4.7 0.3
    ldr r1, =0x001F0300     ;@ 0b0000 0000 0001 1111 0000 0011 0000 0000
    str r1, [r0]

    ldr r0, =XIP_SSI_BAUDR
    ldr r1, =0x00000008
    str r1, [r0]

    ldr r0, =XIP_SSI_SPI_CTRLR0 ;@   2832 2427 2023 1619 12.15 8.11 4.7 0.3
    ldr r1, =0x03000218         ;@ 0b0000 0011 0000 0000 0000 0010 0001 1000
    str r1, [r0]

    ldr r0, =XIP_SSI_CTRLR1
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =XIP_SSI_SSIENR
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r4, =0x10000100     ;@ Source address (FLASH)  ;@ 0x10000100 is the beginning of the vector table
    ldr r5, =0x20000100     ;@ Destination (SRAM)      ;@ copy everything beginning from the vector table to RAM
    ldr r6, =0x1000         ;@ Size of code

_copyToRam:
    ;@ load 16 bytes from FLASH to RAM at a time
    ldmia r4!, {r0-r3}      ;@ load data starting at r4 (auto incrementing) into r0,r1,r2,r3 - Copy data from Flash
    stmia r5!, {r0-r3}      ;@ store data from r0,r1,r2,r3 starting at r5 (auto incrementing) - store data into RAM 
    sub   r6, #16           ;@ 0x1000-16
    bne   _copyToRam        ;@ branch not equal (r6-16=0)

    ;@ Jump to the main function
    ldr r1, =VTOR           ;@ VTOR Register, stores location into r1
    ldr r0, =0x20000100;    ;@ load r0 with value 0x20000100
    ;@ store value in r0 to memory address in r1 - VTOR register = r0 = 0x20000100 (new vector table location)
    str r0, [r1]            

    ldr r0, =0x20002000     ;@ Stack pointer being set to a higher location on the stack (RAM) 
                            ;@ RAM ranges from 0x20000000 to 0x2003FFFF (265KB)
    mov sp, r0

    @; Regardless of the address where the vector table and main function are stored in FLASH (as long as it is before 0x1000100 | 0x1000)
    @; It will be copied into RAM starting from 0x20000100. 
    ldr r0, =0x20000201;    @; Address is odd (0x201) to indicate the instructions are thumb (function at 0x200, and LSB(1) indicates it is thumb)
    bx  r0                  @; branch to the address in r0 (main function)

.set XIP_SSI_BASE,       0x18000000
.set XIP_SSI_CTRLR0,     XIP_SSI_BASE + 0x00
.set XIP_SSI_CTRLR1,     XIP_SSI_BASE + 0x04
.set XIP_SSI_SSIENR,     XIP_SSI_BASE + 0x08
.set XIP_SSI_BAUDR,      XIP_SSI_BASE + 0x14
.set XIP_SSI_SPI_CTRLR0, XIP_SSI_BASE + 0xF4
.set VTOR,               0xE000ED08

.end
