#include <stdlib.h>
#include <stdint.h>

#define PUT32(address, value) (*(volatile uint32_t*) address) = value
#define GET32(address) *(volatile uint32_t*)address

#define MASK(x) (1UL << x)
#define SET (0x2000)
#define CLR (0x3000)

// Resets
#define RESETS_BASE 0x4000c000
#define RESETS_RESET (RESETS_BASE + 0x00)
#define RESETS_RESET_DONE (RESETS_BASE + 0x08)

#define IO_BANK0_BASE 0x40014000
#define IO_BANK0_GPIO00_CTRL (IO_BANK0_BASE + 0x04)
#define IO_BANK0_GPIO01_CTRL (IO_BANK0_BASE + 0x0c)
#define IO_BANK0_GPIO25_CTRL (IO_BANK0_BASE + 0xcc)

#define SPIO_BASE 0xd0000000
#define SIO_GPIO_OE (SPIO_BASE + 0x020)
#define SIO_GPIO_OUT_XOR (SPIO_BASE + 0x01c)
 
#define ROSC_BASE 0x40060000
#define ROSC_STATUS (ROSC_BASE + 0x18)

#define XOSC_BASE 0x40024000
#define XOSC_CTRL (XOSC_BASE + 0x00)
#define XOSC_STATUS (XOSC_BASE + 0x04)
#define XOSC_STARTUP (XOSC_BASE + 0x0c)
#define XOSC_COUNT (XOSC_BASE + 0x1c)

#define CLK_BASE 0x40008000
#define CLK_REF_CTRL (CLK_BASE + 0x30)
#define CLK_REF_DIV (CLK_BASE + 0x34)
#define CLK_SYS_CTRL (CLK_BASE + 0x3c)
#define CLK_SYS_DIV (CLK_BASE + 0x40)
#define CLK_PERI_CTRL (CLK_BASE + 0x48)

#define M0BASE 0xe0000000
#define SYST_CSR (M0BASE + 0xe010)
#define SYST_RVR (M0BASE + 0xe014)
#define SYST_CVR (M0BASE + 0xe018)

#define NVIC_ISER (M0BASE + 0xe100)
#define NVIC_IPR0 (M0BASE + 0xe400)

#define UART0_BASE 0x40034000
#define UART0_DR  (UART0_BASE + 0x000)
#define UART0_FR (UART0_BASE + 0x018)
#define UART0_IBRD (UART0_BASE + 0x024)
#define UART0_FBRD (UART0_BASE + 0x028)
#define UART0_LCR_H (UART0_BASE + 0x02c)
#define UART0_CR (UART0_BASE + 0x030)

#define TIMER_BASE 0x40054000
#define TIMER_ALARM0 (TIMER_BASE + 0x10)
#define TIMER_INTR (TIMER_BASE + 0x34)
#define TIMER_INTE (TIMER_BASE + 0x38)
#define TIMER_INTF (TIMER_BASE + 0x3C)
#define TIMER_INTS (TIMER_BASE + 0x40)

static void irqLoop(void);
static void irqSysTick(void);

/*  
    Deassert reset bit for each subsystem 
    at Initial power-on, subsystems reset is asserted, so we need to deassert the subsystems in 
    use
*/
static void resetSubsys() {
    /* IO Bank holds the configurations for all gpio pins 0-29*/
    PUT32((RESETS_RESET | CLR), MASK(5)); // reset IO_BANK0
    while(GET32(RESETS_RESET_DONE) & MASK(5) == 0); // await reset done signal from IO_BANK0

    /* Reset PADS_BANK0 */
    PUT32((RESETS_RESET | CLR), (MASK(8)));
    while(GET32(RESETS_RESET_DONE) & MASK(8) == 0);

    /* Reset UART0 */
    PUT32((RESETS_RESET | CLR), MASK(22));
    while(GET32(RESETS_RESET_DONE) & MASK(22) == 0);
}

static void init_XOSC() {
    PUT32(XOSC_CTRL, 0xAA0); // Freq_Range set to 1-15 MHz
    PUT32(XOSC_STARTUP, 0xc4 ); // Startup delay ( default value )
    PUT32((XOSC_CTRL | SET), 0xFAB000); // CTRL register offset 0, 0xfab -> ENABLE

    while(!(GET32(XOSC_STATUS) & 0x80000000)); // Oscillator is running and stable

    // SET XOSC as ref, sys and peri clock 
    PUT32(CLK_REF_CTRL, MASK(1)); // CLK_REF source = XOSC_CLKSRC
    PUT32(CLK_SYS_CTRL, 0x0); // CLK SYS source = CLK_REF
    PUT32(CLK_REF_DIV, MASK(8)); // CLK SYS source = CLK_REF
    PUT32(CLK_PERI_CTRL, (MASK(11) | MASK(7))); // MASK(11) -> ENABLE, MASK(7) ->XOSC_CLKSRC
}


static void init_GPIO() {
    // Set GPIO0 and 1 to function 2 (UART0)
    PUT32((IO_BANK0_GPIO00_CTRL), 2);
    PUT32((IO_BANK0_GPIO01_CTRL), 2);

    PUT32(IO_BANK0_GPIO25_CTRL, 0x05); // FUNCSEL = 5 (function 5 from SIO table, 2.19.2)
    PUT32(SIO_GPIO_OE, MASK(25)); // SIO GPIO OE Register (2.3.1.7)
}

/*  Unused method to create timer
    Timer module was planned to be used to generate timed interrupts, but as stated in 
    RP2040 datasheet Appendix B: Errata - RP2040-E7, ROSC and XOSC COUNT registers are 
    unreliable. 

    Cortex M0+ SysTick component/registers were used instead to generate timed interrupts
*/
static void init_timer() {
    #define INT_FREQ_US 12000000/4 // 0.5 seconds at 1us period

    PUT32(TIMER_INTE, MASK(0)); // interupt enable for alarm_0 for timer
    PUT32(NVIC_ISER, MASK(0)); // Enables interrupts for IRQ 0 (IRQ 0 -> TIMER_IRQ_0 on NVIC)
    PUT32(TIMER_ALARM0, INT_FREQ_US); // load value into alarm_0 to trigger interrupt
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
        clk_peri = 12MHz, desired Baud Rate = 115200
        (12 * 10^6) / (16 * 115200) - 6.51
        integer = 6 | fraction = (0.51 (64) + 0.5) = 33
    */
    PUT32(UART0_IBRD, 6);
    PUT32(UART0_FBRD, 33);
    /*  Enable FIFOs, Format */
    PUT32(UART0_LCR_H, (MASK(6) | MASK(5) | MASK(4))); // 8 Bit word Length Enable FIFO
    /*  Set enable Bits in Control Register */
    PUT32(UART0_CR, (MASK(9) | MASK(8) | MASK(0))); // RXE, TXE. UARTEN
}

static unsigned char uartRx(void) {
    while((GET32(UART0_FR) & MASK(4)) != 0); // Wait while the FIFO Register is Empty 
    return((char) GET32(UART0_DR));
}

static void uartTx(unsigned char data) {
    while((GET32(UART0_FR) & MASK(5)) != 0); // Wait while the FIFO Register is full
    PUT32(UART0_DR, data);
}

static void uartTxString(unsigned char* data) {
    while(*data != '\0') {
        uartTx(*data);
        data++;
    }
    
}

/*
    Defines Main function for insertion into memory as defined in the Linker Script
*/
__attribute__( ( used, section( ".boot.entry" ) ) ) int main( void ) {
    // Reset Subsystems (IO / PADS and UART0)
    init_XOSC();
    resetSubsys();
    init_UART();
    init_GPIO();

    unsigned char *start_message = "Hello World";

    #define COUNT_250MS 12000000/4 //12 MHz clk, 12000000 ticks = 1 sec. 250 Ms = 1/4
    PUT32(SYST_RVR, COUNT_250MS); // Load value into counter register
    PUT32(SYST_CSR, (MASK(2) | MASK(1) | MASK(0))); // set to processor clock and start count

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
__attribute__( ( used, section( ".vectors" ) ) ) void ( *vectors[] )( void ) ={
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
    PUT32(SIO_GPIO_OUT_XOR, MASK(25));  // XOR the LED pin
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
        SPI: high speed, on-board, full duplex. used for chip-to-chip communication (flash memory, eeprom ...)
        reset PrimeCell SSP (ARM Synchronous Serial Port architecture)
        must be initd when disabled 
        SSPCLK(clk_peri) max freq is 133MHz
        SSPCR0 and SSPCR1 must be programed to set controller as master or slave under a given protocol 
        bitrate, derived from external SSPCLK, programmed through SSPCPSR register
        can permit FIFO service request to interrupt CPU. transmission and recieve begins on SSPTXD and SSPRXD pins 
        SSPCLK must be <= PCLK
        SSPCLK must be at least 12x max expected freq of SSPCLKIN (external master) - slave mode 
            in master mode, the ratio becomes 2x
            master mode peak bit rate (133MHz) = 62.5Mbps (SSPCPSR programmed with value of 2) and SCR[7:0] in SSPCR0 programmed with 0
            in slave mode, 133/12 = 11.083Mbps. SSPCPSR can be programmed with value of 12 and SCR[7:0] in SSPCR0 can be 0. 

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
            
    */
}