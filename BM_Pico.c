#include <stdlib.h>
#include "BM_Pico.h"

/*  
    Deassert reset bit for each subsystem 
    at Initial power-on, subsystems reset is asserted, so we need to deassert the subsystems in 
    use

    CLR is defined under 2.1.2 Atomic Register Access
        Permits individual fields of a control register to be modified without needing to go through 
        read-modify-write sequence. 
        i.e.    instead of writing RESETS_RESET &= ~(1 << 5) to set bit 5 to 0 we can perform it atomically 
                like: PUT32((RESETS_RESET | CLR), MASK(5));
        this also helps in preventing race conditions and optimizing performance
*/
static void resetSubsys(){}

static void init_PLL() {
    /*  Setup System PLL 
        Configuration: 
                              REF     FBDIV VCO      POSTDIV
            PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHz / 6 / 2 = 125MHz

        Setup Sequence: 
            Program Feedback Divider
            Turn on Main power and VCO
            wait for VCO to lock 
            set up post dividers and turn them on
    */
    RESET_CLR->RESET |= MASK(12); // Reset PLL_SYS
    while(RESETS->RESET_DONE & MASK(12) == 0);

    PLL_SYS->FBDIV_INT = 125; // Set Feedback divisor to 125
    PLL_SYS->PWR &= ~(MASK(5) | MASK(0)); // Turn on the main power and VCO
    while((PLL_SYS->CS & MASK(31)) == 0); // Wait for PLL to lock

    PLL_SYS->PRIM = (MASK_VAL(6, 16) | MASK_VAL(2, 12)); // POSTDIV1 = 6 | POSTDIV2 = 2

    PLL_SYS->PWR &= ~(MASK(3)); // Turn on post Dividers
}
static void init_Clocks() {
    XOSC->CTRL = 0xaa0;
    XOSC->STARTUP = 0xc4;
    XOSC_SET->CTRL = 0xFAB000; // atomic, 0xfab -> ENABLE

    while(!(XOSC->STATUS & 0x80000000)); // Oscillator is running and stable

    init_PLL();

    // SET XOSC as ref, sys and peri clock 
    CLOCK->REF_CTRL = MASK(1); // CLK_REF source = XOSC_CLKSRC
    // Leave CLK_SYS as default (CLKSRC_PLL_SYS -> 125MHz)
    // CLOCK->SYS_CTRL |= MASK(0); // CLK SYS source = CLK_REF
    CLOCK->REF_DIV = MASK(8); // divide by 1
    CLOCK->PERI_CTRL = (MASK(11) | MASK(5)); // MASK(11) -> ENABLE, [7:5] = 001 -> CLKSRC_PLL_SYS (125MHz)
}

static void init_GPIO() {
    /* IO Bank holds the configurations for all gpio pins 0-29*/
    RESET_CLR->RESET |= MASK(5); // reset IO_BANK0
    while(RESETS->RESET_DONE & MASK(5) == 0); // await reset done signal from IO_BANK0

    /* Reset PADS_BANK0 */
    RESET_CLR->RESET |= MASK(8);
    while(RESETS->RESET_DONE & MASK(8) == 0);

    /* Reset UART0 */
    RESET_CLR->RESET |= MASK(22);
    while(RESETS->RESET_DONE & MASK(22) == 0);

    // Set GPIO0 and 1 to function 2 (UART0)
    GPIO0->CTRL = 2;
    GPIO1->CTRL = 2;

    GPIO25->CTRL = 0x05;
    SIO->GPIO_OE = MASK(25);  // SIO GPIO OE Register (2.3.1.7)
}

/*  Unused method to create timer
    Timer module was planned to be used to generate timed interrupts, but as stated in 
    RP2040 datasheet Appendix B: Errata - RP2040-E7, ROSC and XOSC COUNT registers are 
    unreliable. 

    Cortex M0+ SysTick component/registers were used instead to generate timed interrupts
*/
static void init_timer() {
    #define INT_FREQ_US 12000000/4 // 0.5 seconds at 1us period

    TIMER->INTE = MASK(0); // interupt enable for alarm_0 for timer
    CORTEX_M0->NVIC_ISER = MASK(0);  // Enables interrupts for IRQ 0 (IRQ 0 -> TIMER_IRQ_0 on NVIC)
    TIMER->ALARM0 = INT_FREQ_US;// load value into alarm_0 to trigger interrupt
}

static void init_UART() {
    /*
        UART: low speed, good for long distance. full duplex. Simple
        Two UART peripherals
        UART Perfoms serial-to-parallel and vice versa 
        UART operation and baud rate controlled by Line Control Register (UARTLCR_H) and baud rate 
            divisior registers (integer UARTIBRD, fractional UARTFBRD)
        If framing, parity or break error occurs, appropriate error bit is set and stored in FIFO.
        If overrun condition (new byte of data appearing before old one is read from FIFO) occurs, 
            overrun register bit is set and FIFO data is prevented from being overwritten
        32x8 buffer for transmit FIFO and 32x12 for recieve FIFO buffer
        permits DMA and interrupt generation
        Busy signal is set high as soon as data is written to Tx FIFO and persists while data is being
            transmitted. negated only when FIFO is emppty and last char has been trasmitted 

        UARTTXD character frame 
            5-8 data bits 
            parity bit
            1-2 stop bits
    */

    /*  Set baud rate divisor 
        clk_peri = 125 MHz, desired Baud Rate = 115200
        (125 * 10^6) / (16 * 115200) - 67.817
        integer = 67 | fraction = (0.817 (64) + 0.5) = 52
    */
    UART0->IBRD = 67;
    UART0->FBRD = 52;
   
    /*  Enable FIFOs, Format */
    UART0->LCR_H = (MASK(6) | MASK(5) | MASK(4)); // 8 Bit word Length Enable FIFO
    
    /*  Set enable Bits in Control Register */
    UART0->CR = (MASK(9) | MASK(8) | MASK(0)); // RXE, TXE. UARTEN
}

static unsigned char uartRx(void) {
    while((UART0->FR & MASK(4)) != 0); // Wait while the FIFO Register is Empty 
    return((char) UART0->DR);
}

static void uartTx(unsigned char data) {
    while((UART0->FR & MASK(5)) != 0); // Wait while the FIFO Register is full
    UART0->DR = data;
}

static void uartTxString(unsigned char* data) {
    while(*data != '\0') {
        uartTx(*data);
        data++;
    }
}

/*  
    Method to send the contents of a register over uart  
    Primarily used for debugging
*/
static void uartTxRegVal(uint32_t regVal) {
    char hexChars[] = "0123456789ABCDEF"; // List of hex characters to map with
    char hexAsString[9];

    for(int i = 0; i < 8; i++) {
        hexAsString[i] = hexChars[regVal & 0xF]; // get least significant 4 bits 
        regVal >>= 4; // right shift 4
    }

    hexAsString[8] = '\0'; // add string terminator
    uartTxString(hexAsString);
}
/*
    Defines Main function for insertion into memory as defined in the Linker Script
*/
__attribute__((used, section( ".boot.entry" ))) int main(void) {
    init_Clocks();
    // Reset Subsystems (IO / PADS and UART0)
    init_GPIO();
    init_UART();
    init_SPI();

    unsigned char *start_message = "Hello World";

    #define COUNT_250MS 12000000/4 //12 MHz clk, 12000000 ticks = 1 sec. 250 Ms = 1/4
    CORTEX_M0->SYST_RVR = COUNT_250MS;  // Load value into counter register
    CORTEX_M0->SYST_CSR = (MASK(2) | MASK(1) | MASK(0)); // set to processor clock and start count
    uartTxString(start_message);

    while (1) { 
        uartTxString(" --> ");
        uartTx(uartRx()); // Wait for input (bloking function)
        uartTxString("\r\n");
    }

    return (0);
}

/* Vector Table 
    GCC Syntax for specifying an attribute to the compiler
    two attributes defined in this declaration: 
        used:       ensures the function is linked by the compiler, especially since it is not referenced in code  
                    but is an important definition to be included
        section:    creates a section for the compiler to place within the flash memory as defined in the linker script
                    This section is defined as ".vectors" and specifies the handlers for different interrupt routines
*/
__attribute__((used, section(".vectors"))) void (*vectors[])(void) ={
    0,          //  0 stack pointer value (NA)
    irqLoop,    //  1 reset (NA)
    irqLoop,    //  2 NMI
    irqLoop,    //  3 hardFault
    0,          //  4 reserved
    0,          //  5 reserved
    0,          //  6 reserved
    0,          //  7 reserved
    0,          //  8 reserved
    0,          //  9 reserved
    0,          // 10 reserved
    irqLoop,    // 11 SVCall
    0,          // 12 reserved
    0,          // 13 reserved
    irqLoop,    // 14 pendSV
    irqSysTick, // 15 sysTick
};

/* Handle unconfigured interrupts*/
static void irqLoop(void) {
    while (1);
}

/* Handler of the Tick interrupt */
static void irqSysTick(void) {
    SIO->GPIO_OUT_XOR = MASK(25);  // XOR the LED pin
}


static void init_I2C() {
    /*
        I2C: low speed, on-board. chip-to-chip communication with simple sensors
    */
}

static void init_USB() {
    /*
        controller requires clk_usb to run at 48MHz, and sys_clk must be >- 48 MHz
        Max data rate for USB full speed is 12Mbps
        4kB of DPSRAM (dual port SRAM) - used to store controll registers and data buffers 
            - 32-bit wide memory at address 0 of USB controler (0x50100000)
        DPSRAM should be considered asynchronous and not atomic 
            - processor and usb can both access (read/wrtite) at the same time as eachother
            - available lit in buffer control register is used to indicate who has ownership 
            - Process
                - write buffer information to buffer control register
                - nop (no op) for some clk_sys cycles to ensure that atleast one clk_usb cycle has passed
                    - f_clk_sys/F_clk_usb = how many clk_sys cycles to nop for 
                - set Availbale bit 
    */
}

/* no need to init SPI for communication with PC, we have USB peripheral to do so*/
static void init_SPI() {
    /* 
        4 wires to communicate between master and slave 
            - SCLK - used to synchronize
            - MOSI (master out slave in)
            - MISO (master in slave out)
            - CS (chip select) - what peripheral we are communicating with 

        SPI: high speed, on-board, full duplex. used for chip-to-chip communication (flash memory, eeprom ...)
        reset PrimeCell SSP (ARM Synchronous Serial Port architecture)
        must be initd when disabled 
        SSPCLK(clk_peri) max freq is 133MHz
        PCLK (clk_sys)
        SSPCR0 and SSPCR1 must be programed to set controller as master or slave under a given protocol 
        bitrate, derived from external SSPCLK, programmed through SSPCPSR register
        can permit FIFO service request to interrupt CPU. transmission and recieve begins on SSPTXD and SSPRXD pins 
        SSPCLK must be <= PCLK
        SSPCLK rates:
            master mode peak bit rate (133MHz) = 62.5Mbps (SSPCPSR programmed with value of 2) and SCR[7:0] in SSPCR0 programmed with 0
            in slave mode, 133/12 = 11.083Mbps. SSPCPSR can be programmed with value of 12 and SCR[7:0] in SSPCR0 can be 0. 

        In Master Mode, the SSPCLK to SSPCLKOUT frequency ratio is at minimum 2x
        Because of setup and hold times on SSPRXD, SSPCLK must be atleast 12x faster than Maximum expected Hz of SSPCLKIN
            - LSM6DSOX SPI Max CLK freq = 10MHz (4.4.1 of LSM6DSOX datasheet)
            - Therefor, SSPCLK must be atleast 120 Mhz 

        Control Registers: 
            - SSPCR0
                - Program Serial Clk rate 
                - select one of 3 protocols 
                - select data word size 
            - SSPCR1
                - Select master or slave mode 
                - enable loop back test feature 
                - enable PrimeCell SSP peripheral 

        Bit rate: 
            - SSPCLK / CPSDVSR * (1 + SCR)
                - SSPCLK (clk_peri)
                - CPSDVSR - prescale value in range of 2-254 (programmed in SSPCPSR)
                - SCR - value programmed in SSPCR0
            
        Frame Format
            - 4-16 bit length (size is programmed)
            - MSB is first transmitted 
            - SSPCLKOUT is inactive while SSP is idle, and becomes active when data is transmitted or recieved 
            - 
            
        Configuration: 
            - Reset 
            - configure SSPCR0 and SSPCR1 

    */    
    /* Reset SPI0 */
    RESET_CLR->RESET |= MASK(16);
    while(RESETS->RESET_DONE & MASK(16) == 0);

    // Init while disabled 
    SPI0->SSPCR0 = (MASK(4) | MASK(2) | MASK(1) | MASK(0)); // TI Synchronous Serial Frame Format, 8-bit data
    SPI0->SSPCR1 = (MASK(0)); // LoopBack mode Enable for Testing 



    SPI0->SSPCR1 = (MASK(1)); // Enable SSP operation 

}