#include <stdlib.h>
#include <stdint.h>

#define PUT32(address, value) (*(volatile uint32_t*) address) = value
#define GET32(address) *(volatile uint32_t*)address

#define MASK(x) (1UL << x)
#define CLR (0x3000)

// Resets
#define RESETS_BASE 0x4000C000
#define RESETS_RESET (RESETS_BASE + 0x00)
#define RESETS_RESET_DONE (RESETS_BASE + 0x08)

#define SPIO_BASE 0xd0000000
#define GPIO25_CTRL 0x0CC
#define GPIO25_STATUS 0x0C8
#define SIO_GPIO_OE (SPIO_BASE + 0x020)
#define SIO_GPIO_OUT_XOR (SPIO_BASE + 0x01C)
 
#define ROSC_BASE 0x40060000
#define ROSC_STATUS (ROSC_BASE + 0x18)

#define XOSC_BASE 0x40024000
#define XOSC_STATUS (XOSC_BASE + 0x04)
#define XOSC_STARTUP (XOSC_BASE + 0x0C)
#define XOSC_COUNT (XOSC_BASE + 0x1C)

#define CLK_BASE 0x40008000
#define CLK_REF_CTRL (CLK_BASE + 0x30)
#define CLK_REF_DIV (CLK_BASE + 0x34)
#define CLK_SYS_CTRL (CLK_BASE + 0x3C)
#define CLK_SYS_DIV (CLK_BASE + 0x40)
#define CLK_PERI_CTRL (CLK_BASE + 0x54)

#define M0BASE 0xe0000000
#define SYST_CSR (M0BASE + 0xE010)
#define SYST_RVR (M0BASE + 0xE014)
#define SYST_CVR (M0BASE + 0xE018)

#define NVIC_ISER (M0BASE + 0xE100)
#define NVIC_IPR0 (M0BASE + 0xE400)

#define TIMER_BASE 0x40054000
#define TIMER_ALARM0 (TIMER_BASE + 0x10)
#define TIMER_INTR (TIMER_BASE + 0x34)
#define TIMER_INTE (TIMER_BASE + 0x38)
#define TIMER_INTF (TIMER_BASE + 0x3C)
#define TIMER_INTS (TIMER_BASE + 0x40)

/* Handle unconfigured interrupts*/
void irqLoop(void) {
    while (1);
}

/* Handler of the Tick interrupt */
void irqSysTick(void) {
    PUT32(SIO_GPIO_OUT_XOR, MASK(25));  // XOR the LED pin
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

void init_XOSC() {
    PUT32(XOSC_BASE, 0xAA0); // Freq_Range set to 1-15 MHz
    PUT32(XOSC_STARTUP, 0xc4 ); // Startup delay ( default value )
    PUT32(XOSC_BASE, 0xFAB000); // CTRL register offset 0, 0xfab -> ENABLE

    while(!(GET32(XOSC_STATUS) & 0x80000000)); // Oscillator is running and stable

    // SET XOSC as ref, sys and peri clock 
    PUT32(CLK_REF_CTRL, 0x2); // CLK_REF source = XOSC_CLKSRC
    PUT32(CLK_SYS_CTRL, 0x0); // CLK SYS source = CLK_REF
    PUT32(CLK_PERI_CTRL, (MASK(11) | MASK(7))); // MASK(11) -> ENABLE, MASK(7) ->XOSC_CLKSRC
}

/* reset the subsystems used in this program */
static void resetSubsys() {
    /* IO Bank holds the configurations for all gpio pins 0-29*/
    PUT32((RESETS_RESET | CLR), MASK(5)); // reset IO_BANK0
    while(GET32(RESETS_RESET_DONE) & MASK(5) == 0); // await reset done signal from IO_BANK0
}

/*  Unused method to create timer
    Timer module was planned to be used to generate timed interrupts, but as stated in 
    RP2040 datasheet Appendix B: Errata - RP2040-E7, ROSC and XOSC COUNT registers are 
    unreliable. 

    Cortex M0+ SysTick component/registers were used instead to generate timed interrupts
*/
void init_timer() {
    #define INT_FREQ_US 12000000/4 // 0.5 seconds at 1us period

    PUT32(TIMER_INTE, MASK(0)); // interupt enable for alarm_0 for timer
    PUT32(NVIC_ISER, MASK(0)); // Enables interrupts for IRQ 0 (IRQ 0 -> TIMER_IRQ_0 on NVIC)
    PUT32(TIMER_ALARM0, INT_FREQ_US); // load value into alarm_0 to trigger interrupt
}


/*
    Defines Main function for insertion into memory as defined in the Linker Script
*/
__attribute__( ( used, section( ".boot.entry" ) ) ) int main( void ) {
    // IO User Bank base addr 0x40014000 (2.19.6.1)
    init_XOSC();
    // Reset Subsystems (IO / PADS and UART0)
    resetSubsys();

    PUT32((0x40014000 | GPIO25_CTRL), 0x05); // FUNCSEL = 5 (function 5 from SIO table, 2.19.2)
    PUT32(SIO_GPIO_OE, MASK(25)); // SIO GPIO OE Register (2.3.1.7)

    #define COUNT_250MS 12000000/4 //12 MHz clk, 12000000 ticks = 1 sec. 250 Ms = 1/4
    PUT32(SYST_RVR, COUNT_250MS); // Load value into counter register
    PUT32(SYST_CSR, (MASK(2) | MASK(1) | MASK(0))); // set to processor clock and start count

    while (1) {
        // loop here and wait for the tick interrupt
    }

    return (0);
}
