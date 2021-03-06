#define UART0_IBRD      ((volatile unsigned int*)(0x3F201024))
#define UART0_FBRD      ((volatile unsigned int*)(0x3F201028))
#define UART0_LCRH      ((volatile unsigned int*)(0x3F20102C))
#define UART0_CR        ((volatile unsigned int*)(0x3F201030))
#define UART0_ICR       ((volatile unsigned int*)(0x3F201044))
#define GPFSEL1         ((volatile unsigned int*)(0x3F200004))
#define GPPUD           ((volatile unsigned int*)(0x3F200094))
#define GPPUDCLK0       ((volatile unsigned int*)(0x3F200098))
#define UART0_DR        ((volatile unsigned int*)(0x3F201000))
#define UART0_FR        ((volatile unsigned int*)(0x3F201018))

void uart_init_boot() {

    register unsigned int r;

    /* initialize UART */
    *UART0_CR = 0;      // turn off UART0

    /* map UART0 to GPIO pins */
    r = *GPFSEL1;
    r &= ~((7 << 12) | (7 << 15)); // gpio14, gpio15
    r |= (4 << 12) | (4 << 15);    // alt0
    *GPFSEL1 = r;
    *GPPUD = 0;            // enable pins 14 and 15
    r = 150;
    while (r--) { asm volatile("nop"); }
    *GPPUDCLK0 = (1 << 14) | (1 << 15);
    r = 150;
    while (r--) { asm volatile("nop"); }
    *GPPUDCLK0 = 0;        // flush GPIO setup

    *UART0_ICR = 0x7FF;    // clear interrupts
    *UART0_IBRD = 2;       // 115200 baud
    *UART0_FBRD = 0xB;
    *UART0_LCRH = 0b11 << 5; // 8n1
    *UART0_CR = 0x301;     // enable Tx, Rx, FIFO
}

/**
 * Send a character
 */
void uart_send_boot(unsigned int c) {
    /* wait until we can send */
    do { asm volatile("nop"); } while (*UART0_FR & 0x20);
    /* write the character to the buffer */
    *UART0_DR = c;
}

/**
 * Receive a character
 */
char uart_getc_boot() {
    char r;
    /* wait until something is in the buffer */
    do { asm volatile("nop"); } while (*UART0_FR & 0x10);
    /* read it and return */
    r = (char)(*UART0_DR);
    /* convert carrige return to newline */
    return r == '\r' ? '\n' : r;
}

/**
 * Display a string
 */
void uart_puts(char *s) {
    while(*s) {
        /* convert newline to carrige return + newline */
        if(*s=='\n')
            uart_send('\r');
        uart_send(*s++);
    }
}

/**
 * Display a binary value in hexadecimal
 */
void uart_hex(unsigned int d) {
    unsigned int n;
    int c;
    for(c=28;c>=0;c-=4) {
        // get highest tetrad
        n=(d>>c)&0xF;
        // 0-9 => '0'-'9', 10-15 => 'A'-'F'
        n+=n>9?0x37:0x30;
        uart_send(n);
    }
}


