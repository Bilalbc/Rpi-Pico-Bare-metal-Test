#include <stdlib.h>
#include <stdbool.h>
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
    GPIO0->CTRL = 0x2;
    GPIO1->CTRL = 0x2;

    GPIO25->CTRL = 0x05;
    SIO->GPIO_OE = MASK(25);  // SIO GPIO OE Register (2.3.1.7)

    /* Configure SPI0 Pins to F1*/
    GPIO16->CTRL |= 0x1;
    GPIO17->CTRL |= 0x1;
    GPIO18->CTRL |= 0x1;
    GPIO19->CTRL |= 0x1;

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

        Interrupts: 
            - TxINTR : Transmit FIFO interrupt
            - RxINTR : Recieve FIFO interrupt
            - RTINTR : Recieve Trimeout interrupt
            - RORINTR : Recieve Overrun interrupt
            all are default to masked interrupts

    */    
    /* Reset SPI0 */
    RESET_CLR->RESET |= MASK(16);
    while(RESETS->RESET_DONE & MASK(16) == 0);

    // Init while disabled 
    SPI0->SSPCR0 = (MASK(2) | MASK(1) | MASK(0)); // Motorola SPI Frame Format, 8-bit data
    SPI0->SSPCR1 |= (MASK(0)); // LoopBack mode Enable for Testing 

    /*
        SSPCLK(min) >= 2 x SSPCLKOUT(max) in Master
        SSPCLK(max) <= 254 * 256 * SSPCLKOUT(min) in Master
    */

    SPI0->SSPCPSR = MASK_VAL(2, 0); // Configure peak bit rate for master mode (2-254)
    // SCR value is unchanged from 0

    GPIO17->CTRL |= (MASK_VAL(0x3, 12)); // OEOVER - Enable Output
    GPIO17->CTRL |= (MASK_VAL(0x3, 8));  // OUTOVER - Drive output high

    SPI0->SSPCR1 |= MASK(1); // Enable SSP operation 
}

static void init_LSM6DSOX() {

    /*  LSM6DSOX SPI Config from Datasheet
        
        4 wires:
            - CS    : Chip Select - Enables Serial Port, low at start of transmission and high when finished
            - SPC   : Serial Port Clock - Controlled by SPI Master, is stopped high when CS is high (no transmission)
            - SDI   : Serial Data Input - falling edge of SPC
            - SDO   : Serial Data Output - falling edge of SPC

        Data format (bits) (MSB always first)
            - 0     : RW bit. 0 = data DI(7:0) is written into device. 1 = Data DI(7:0) from device is read
                - in latter case, SDO is driven at the start of bit 8
            - 1-7   : Address AD(6:0). addressed field of indexed register
            - 8-15  : DO(7:0)/DI(7:0) (r/w modes)

    */
   
    /* Initialize LSM6DOSX Accelerometer */
    

    /* Test setup by reading LSM6DSOX WHO_AM_I register - expected value 0x6C */
}

/*  Method to create an SPI Write request to the LSM6DSOX sensor 
    16 Bit format: 
        - RW bit (1 for write request)
        - 7 bit address frame
        - 8 bit data frame 
    LSB for reg must be 0 in this case so that it can be overwritten by data 

    helpful resources: https://forums.raspberrypi.com/viewtopic.php?t=95511
    https://leibton.medium.com/communication-between-lsm6dsox-and-raspberry-pi-through-spi-97fe2e30432b 
    https://www.digikey.ca/en/maker/projects/raspberry-pi-pico-rp2040-spi-example-with-micropython-and-cc/9706ea0cf3784ee98e35ff49188ee045 
*/
static void createSPIWriteReq(unsigned char* request, short reg, char data) {
    request[0] = (0x00 | reg); //MSB = 0, register address (8-14)
    request[1] = data; // add Data to write (0-7)
    uartTxSerialPacket(request[0],request[1]);
}

static void createSPIReadReq(unsigned char* request, char reg) {
    request[0] = 0x80 | reg; //MSB = 1, register address (8-14)
    request[1] = 0x00; 
}

static unsigned char spiRx(void) {
    while((SPI0->SSPSR & MASK(2)) == 0);
    return SPI0->SSPDR;
}

static void spiTx(unsigned char* request) {
    while((SPI0->SSPSR & MASK(1)) == 0);
    uartTxSerialPacket(request[0],request[1]);
    SPI0->SSPDR = (0x00ff & request[0]); // right justify data for data less than 16 bits
    SPI0->SSPDR = (0x00ff & request[1]); // right justify data for data less than 16 bits
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

    for(int i = 7; i >= 0; i--) {
        hexAsString[i] = hexChars[regVal & 0xF]; // get least significant 4 bits 
        regVal >>= 4; // right shift 4
    }

    hexAsString[8] = '\0'; // add string terminator
    uartTxString(hexAsString);
}

/*  
    Method to send the contents of a serial packet over uart
    Primarily used for debugging
*/
static void uartTxSerialPacket(unsigned char packet_upper, unsigned char packet_lower) {
    char hexChars[] = "0123456789ABCDEF"; // List of hex characters to map with
    unsigned char packetAsString[5];

    packetAsString[0] = hexChars[(packet_upper & 0xF0) >> 4]; // get the upper nibble
    packetAsString[1] = hexChars[packet_upper & 0xF]; // get the lower nibble

    packetAsString[2] = hexChars[(packet_lower & 0xF0) >> 4]; // get the upper nibble
    packetAsString[3] = hexChars[packet_lower & 0xF]; // get the lower nibble

    packetAsString[4] = '\0'; // add string terminator
    uartTxString(packetAsString);
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

    // expected:    0b0001 1111
    // actual:      0b0111 1110

    unsigned char request[2];
    createSPIWriteReq(request, 0x7f, 0x8e);
    while (1) { 
        uartTxString(" --> ");
        uartTxString(" SENDING 'a' over SPI: ");
        spiTx(request);
        uartTxString("\r\n");
        uartTxString(" RECIEVING over SPI: ");
        uartTxRegVal(spiRx());
        uartTxString("\r\n");
        uartTx(uartRx()); // Wait for input (bloking function)
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

    void (*vectors[])(void) - Defomes 'vectors' as an array of function pointers that take no args and return void

    when an expcetion occurs, the VTOR reg is used to reference this table, and then the process skips to the function 
    defined by this table given the interrupt source. if the function is 0, we attempt to branch to address 0x00000000
    which is non valid executable memory and so causes a HardFault exception 

    VTOR Reg points to this table, as defined in memmap.ld and boot2.s
    
    Exceptions 0-15 processor related exceptions
    Excpetions 16-25 are peripheral interrupts
*/
__attribute__((used, section(".vectors"))) void (*vectors[])(void) = {
    /* Exceptions 0-15*/
    0,                  //  0 stack pointer value (NA)
    irqLoop,            //  1 reset (NA)
    irqLoop,            //  2 NMI
    irqHardFault,       //  3 hardFault
    0,                  //  4 reserved
    0,                  //  5 reserved
    0,                  //  6 reserved
    0,                  //  7 reserved
    0,                  //  8 reserved
    0,                  //  9 reserved
    0,                  // 10 reserved
    irqLoop,            // 11 SVCall
    0,                  // 12 reserved
    0,                  // 13 reserved
    irqLoop,            // 14 pendSV
    irqSysTick,         // 15 sysTick
    /* External Interrupts */
    irqLoop,            //  0 TIMER_IRQ_0 (16)
    irqLoop,            //  1 TIMER_IRQ_1
    irqLoop,            //  2 TIMER_IRQ_2
    irqLoop,            //  3 TIMER_IRQ_3
    irqLoop,            //  4 PWM_IRQ_WRAP
    irqLoop,            //  5 USBCTRL_IRQ
    irqLoop,            //  6 XIP_IRQ
    irqLoop,            //  7 PIO0_IRQ_0
    irqLoop,            //  8 PIO0_IRQ_1
    irqLoop,            //  9 PIO1_IRQ_0
    irqLoop,            // 10 PIO1_IRQ_1
    irqLoop,            // 11 DMA_IRQ_0
    irqLoop,            // 12 DMA_IRQ_1
    irqLoop,            // 13 IO_IRQ_BANK0
    irqLoop,            // 14 IO_IRQ_QSPI
    irqLoop,            // 15 SIO_IRQ_PROC0
    irqLoop,            // 16 SIO_IRQ_PROC1
    irqLoop,            // 17 CLOCKS_IRQ
    irqLoop,            // 18 SPI0_IRQ
    irqLoop,            // 19 SPI1_IRQ
    irqLoop,            // 20 UART0_IRQ
    irqLoop,            // 21 UART1_IRQ
    irqLoop,            // 22 ADC_IRQ_FIFO
    irqLoop,            // 23 I2C0_IRQ
    irqLoop,            // 24 I2C1_IRQ
    irqLoop,            // 25 RTC_IRQ 
};

/* Handle Hard Fault */
static void irqHardFault(void) {
    uartTxString("HARDFAULT\n");
    while(1);
}

/* Handle unconfigured interrupts*/
static void irqLoop(void) {
    uartTxString("LOOPING\n");
    SIO->GPIO_OUT_XOR = MASK(25);  // XOR the LED pin
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
