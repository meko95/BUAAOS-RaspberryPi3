#define UART0_IBRD	((volatile u_int*)(0x3F201024))
#define UART0_FBRD	((volatile u_int*)(0x3F201028))
#define UART0_LCRH	((volatile u_int*)(0x3F20102C))
#define UART0_CR 	((volatile u_int*)(0x3F201030))	
#define UART0_ICR	((volatile u_int*)(0x3F201044))
#define GPFSEL1 	((volatile u_int*)(0x3F200004))
#define GPPUD 		((volatile u_int*)(0x3F200094))
#define GPPUDCLK0 	((volatile u_int*)(0x3F200098))
#define UART0_DR 	((volatile u_int*)(0x3F201000))
#define UART0_FR 	((volatile u_int*)(0x3F201018))

void uart_init() {
	register unsigned int r;
	*UART0_CR = 0;
	r = *GPFSEL1;
	r &= ~((7 << 12) | (7 << 15)); // gpio14, gpio15 
	r |= (4<<12)|(4<<15); //alt0
	*GPFSEL1 = r;
	*GPPUD = 0; // enable pins 14 and 15 
	r = 150;
	while (r--) { asm volatile("nop"); }
	*GPPUDCLK0 = (1 << 14) | (1 << 15);
	r = 150;
	while (r--) { asm volatile("nop"); }
	    *GPPUDCLK0 = 0;
	    *UART0_ICR = 0x7FF;
	    *UART0_IBRD = 2;
	    *UART0_FBRD = 0xB;
	    *UART0_LCRH = 0b11 << 5; // 8n1
	    *UART0_CR = 0x301;     // enable Tx, Rx, FIFO
}
void uart_send(unsigned int c) {
	do { asm volatile("nop"); } while (*UART0_FR & 0x20);
    *UART0_DR = c;
}
