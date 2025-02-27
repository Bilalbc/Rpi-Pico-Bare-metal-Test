#ifndef BM_Pico.H
#define BM_PICO.H

#include <stdint.h>

#define PUT32(address, value) (*(volatile uint32_t*) address) = value
#define GET32(address) *(volatile uint32_t*)address
#define MASK(x) (1UL << x)
#define MASK_VAL(x, y) (x << y)

#define SET             0x2000
#define CLR             0x3000

/* Sub System Registers*/
#define IO_BANK0_BASE   0x40014000
typedef struct {
    volatile uint32_t   STATUS;           // Status Register  offset: 0x00
    volatile uint32_t   CTRL;             // Control Register  offset: 0x04
} IO_BANK_t;

#define RESETS_BASE     0x4000c000
typedef struct {
    volatile uint32_t   RESET;            // 0ffset: 0x00
    volatile uint32_t   WDSEL;            // 0ffset: 0x04
    volatile uint32_t   RESET_DONE;       // 0ffset: 0x08
} RESETS_t;

#define SIO_BASE        0xd0000000
typedef struct {
    volatile uint32_t   UNUSED[7];        // Registers unused in this program: 0x00-0x18
    volatile uint32_t   GPIO_OUT_XOR;     // 0ffset: 0x01c
    volatile uint32_t   GPIO_OE;          // 0ffset: 0x020
} SIO_t;

#define XOSC_BASE       0x40024000
typedef struct {
    volatile uint32_t   CTRL;             // 0ffset: 0x00
    volatile uint32_t   STATUS;           // 0ffset: 0x04
    volatile uint32_t   DORMANT;          // 0ffset: 0x08
    volatile uint32_t   STARTUP;          // 0ffset: 0x0c
    volatile uint32_t   RESERVED[3];      // Reserved: 0x10 - 0x18
    volatile uint32_t   COUNT;            // 0ffset: 0x1c
} XOSC_t;

#define CLK_BASE        0x40008000
typedef struct {
    volatile uint32_t   UNUSED[12];       // Unused in this program: 0x00-0x2c
    volatile uint32_t   REF_CTRL;         // 0ffset: 0x30       
    volatile uint32_t   REF_DIV;          // 0ffset: 0x34
    volatile uint32_t   REF_SELECTED;     // 0ffset: 0x38
    volatile uint32_t   SYS_CTRL;         // 0ffset: 0x3c
    volatile uint32_t   SYS_DIV;          // 0ffset: 0x40
    volatile uint32_t   SYS_SELECTED;     // 0ffset: 0x44
    volatile uint32_t   PERI_CTRL;        // 0ffset: 0x48
    volatile uint32_t   PERI_DIV;         // 0ffset: 0x4c
    volatile uint32_t   PERI_SELECTED;    // 0ffset: 0x50
} CLOCK_t;

#define UART0_BASE      0x40034000
typedef struct {
    volatile uint32_t   DR;               // 0ffset: 0x000 
    volatile uint32_t   RSR;              // 0ffset: 0x004
    volatile uint32_t   RESERVED[4];      // Reserved: 0x008-0x014
    volatile uint32_t   FR;               // 0ffset: 0x018 
    volatile uint32_t   RESERVED1;        // Reserved: 0x001c
    volatile uint32_t   ILPR;             // 0ffset: 0x020
    volatile uint32_t   IBRD;             // 0ffset: 0x024 
    volatile uint32_t   FBRD;             // 0ffset: 0x028
    volatile uint32_t   LCR_H;            // 0ffset: 0x02c
    volatile uint32_t   CR;               // 0ffset: 0x030
} UART_t;

#define M0BASE          0xe0000000
typedef struct {
    volatile uint32_t   RESERVED[14340];  // Reserved: 0x0000 - 0xe010
    volatile uint32_t   SYST_CSR;         // 0ffset: 0xe010
    volatile uint32_t   SYST_RVR;         // 0ffset: 0xe014
    volatile uint32_t   SYST_CVR;         // 0ffset: 0xe018
    volatile uint32_t   RESERVED_1[57];    // Reserved: 0xe01c - 0xe0fc
    volatile uint32_t   NVIC_ISER;        // Offset: 0xe100
    volatile uint32_t   RESERVED_2[31];    // Reserved: 0xe104 - 0xe180
    volatile uint32_t   NVIC_ICER;        // Offset: 0xe180
    volatile uint32_t   RESERVED_3[31];    // Reserved: 0xe184 - 0xe200
    volatile uint32_t   NVIC_ISPR;        // Offset: 0xe200
    volatile uint32_t   RESERVED_4[31];    // Reserved: 0xe204 - 0xe280
    volatile uint32_t   NVIC_ICPR;        // Offset: 0xe280
} CORTEX_M0_t;

#define TIMER_BASE      0x40054000
typedef struct {
    volatile uint32_t   UNUSED[4];        // Unused: 0x00 - 0x0c
    volatile uint32_t   ALARM0;           // 0ffset: 0x10
    volatile uint32_t   UNUSED_1[8];       // Unused: 0x14 - 0x30
    volatile uint32_t   INTR;             // 0ffset: 0x34
    volatile uint32_t   INTE;             // 0ffset: 0x38
    volatile uint32_t   INTF;             // 0ffset: 0x3c
    volatile uint32_t   INTS;             // 0ffset: 0x40
} TIMER_t;

#define SPI0_BASE       0x4003c000
typedef struct {
    volatile uint32_t   SSPCR0;           // 0ffset: 0x000
    volatile uint32_t   SSPCR1;           // 0ffset: 0x004
    volatile uint32_t   SSPDR;            // 0ffset: 0x008
    volatile uint32_t   SSPSR;            // 0ffset: 0x00c
    volatile uint32_t   SSPCPSR;          // 0ffset: 0x010
    volatile uint32_t   SSPIMSC;          // 0ffset: 0x014
    volatile uint32_t   SSPRIS;           // 0ffset: 0x018
    volatile uint32_t   SSPMIS;           // 0ffset: 0x01c
    volatile uint32_t   SSPICR;           // 0ffset: 0x020
} SSP_t;

#define PLL_SYS_BASE    0x40028000     
typedef struct{
    volatile uint32_t   CS;               // 0ffset: 0x0
    volatile uint32_t   PWR;              // 0ffset: 0x4
    volatile uint32_t   FBDIV_INT;        // 0ffset: 0x8
    volatile uint32_t   PRIM;             // 0ffset: 0xc
} PLL_t;

/* GPIO Pins */
#define GPIO0 ((IO_BANK_t*) (IO_BANK0_BASE + 0x000)) // UART0 Tx
#define GPIO1 ((IO_BANK_t*) (IO_BANK0_BASE + 0x008)) // UART0 Rx
#define GPIO16 ((IO_BANK_t*) (IO_BANK0_BASE + 0x080)) // SPI0 Rx (MISO)
#define GPIO17 ((IO_BANK_t*) (IO_BANK0_BASE + 0x088)) // SPI0 CSn
#define GPIO18 ((IO_BANK_t*) (IO_BANK0_BASE + 0x090)) // SPI0 SCK
#define GPIO19 ((IO_BANK_t*) (IO_BANK0_BASE + 0x098)) // SPI0 Tx (MOSI)
#define GPIO25 ((IO_BANK_t*) (IO_BANK0_BASE + 0x0c8))

/* Sub System Initializations */

#define RESETS ((RESETS_t*) RESETS_BASE)
#define RESET_CLR ((RESETS_t*) (RESETS_BASE | CLR))
#define RESET_SET ((RESETS_t*) (RESETS_BASE | SET))

#define SIO ((SIO_t*) SIO_BASE)

#define SPI0 ((SSP_t*) SPI0_BASE)

#define XOSC ((XOSC_t*) XOSC_BASE)
#define XOSC_SET ((XOSC_t*) (XOSC_BASE | SET))

#define CLOCK ((CLOCK_t*) CLK_BASE)

#define UART0 ((UART_t*) UART0_BASE)

#define CORTEX_M0 ((CORTEX_M0_t*) M0BASE)
#define TIMER ((TIMER_t*) TIMER_BASE)

#define PLL_SYS ((PLL_t*) PLL_SYS_BASE)

/* Flag Registers */

/* Functions */
static void irqHardFault(void);
static void irqLoop(void);
static void irqSysTick(void);

static void uartTxString(unsigned char* data);
static void uartTx(unsigned char data);
static unsigned char uartRx(void);
static void uartTxRegVal(uint32_t regVal);
static void uartTxSerialPacket(unsigned char packet_upper, unsigned char packet_lower);

#endif