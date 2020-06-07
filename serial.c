#include "serial.h"
#include "stm32f411_irqn.h"

void serial_wait(struct USART1_Type* usart) {
	while (usart->CR1 & USART1_CR1_TXEIE)
		__NOP();
}

struct Ringbuffer {
	uint8_t* buf;
	uint16_t bufszmsk; // size must be power of two, this is the mask 
	uint16_t head;     // writes happen here
	uint16_t tail;     // reads happen here
};

static inline uint16_t ringbuffer_avail(struct Ringbuffer* rb) { return rb->head - rb->tail; }                  // 0..size -1
static inline uint16_t ringbuffer_free(struct Ringbuffer* rb)  { return rb->bufszmsk - (rb->head - rb->tail); } // size-1 .. 0
static inline int      ringbuffer_empty(struct Ringbuffer* rb) { return rb->head == rb->tail; }
static inline int 	   ringbuffer_full(struct Ringbuffer* rb) { return ringbuffer_free(rb) < 2; }
static inline void     put_head(struct Ringbuffer* rb, uint8_t c) { rb->buf[rb->head++ & rb->bufszmsk] = c; }


static struct Ringbuffer tx_buf[3];

static inline int usart_index(struct USART1_Type* usart) {
	if (usart == &USART1)
		return 0;
	if (usart == &USART2)
		return 1;
	if (usart == &USART6)
		return 2;

	for (;;)
		__NOP(); // hang

	return 0;
}

void serial_init(struct USART1_Type* usart, int baud, uint8_t* txbuf, uint16_t bufsz) {
	const int idx = usart_index(usart);

	if (bufsz & (bufsz-1)) {
		for (;;)
			__NOP(); // hang
	}

	tx_buf[idx].buf = txbuf;
	tx_buf[idx].bufszmsk = bufsz-1;
	tx_buf[idx].head = 0;
	tx_buf[idx].tail = 0;

	usart->CR1 = 0;
	usart->CR2 = 0;
	usart->CR3 = 0;

	// PCLK2 for USART1, PCLK1 for USART 2, 3
	// this depends on SetSystemclockTo96MHz setting pclk1 to sysclk/2 and pclk1 to sysclc/2
	uint32_t clk = 96000000;
	if (usart == &USART2) {
		clk /= 2;
	}
	usart->BRR = clk / baud;

	static const enum IRQn_Type irqn[3] = {IRQ_USART1,IRQ_USART2,IRQ_USART6};
	IRQ_Enable(irqn[idx]);

	usart->CR1 = USART1_CR1_UE | USART1_CR1_TE;
}

static inline void irqHandler(struct USART1_Type* usart, struct Ringbuffer* rb) {
	if (!ringbuffer_empty(rb)) {
		if (usart->SR & USART1_SR_TXE) {
			usart->DR = rb->buf[rb->tail++ & rb->bufszmsk];
		}
	} else {
		usart->CR1 &= ~USART1_CR1_TXEIE;
	}	
	return;
}

// IRQ Handlers end up in the IRQ vector table via startup_*.s
void USART1_Handler(void) { irqHandler(&USART1, &tx_buf[0]); }
void USART2_Handler(void) { irqHandler(&USART2, &tx_buf[1]); }
void USART6_Handler(void) { irqHandler(&USART6, &tx_buf[2]); }


#define STB_SPRINTF_STATIC
#define STB_SPRINTF_MIN 32
#define STB_SPRINTF_NOFLOAT
#define STB_SPRINTF_IMPLEMENTATION

#include "stb_sprintf.h"

static char* rb_putcb(char* buf, void* user, int len) {
	struct Ringbuffer* rb = (struct Ringbuffer*)user;
	if (ringbuffer_free(rb) < len) {
		rb->head = rb->tail;
		put_head(rb, '!');
		put_head(rb, 'O');
		put_head(rb, 'V');
		put_head(rb, 'F');
		put_head(rb, '!');
	}
	for (int i = 0; i < len; ++i) {
		put_head(rb, buf[i]);
	}
	return buf;
}

int serial_printf(struct USART1_Type* usart, const char* fmt, ...) {
	const int          idx = usart_index(usart);
	struct Ringbuffer* tx  = &tx_buf[idx];
	stbsp_set_separators('\'', '.');

	va_list ap;
	va_start(ap, fmt);
	char b[STB_SPRINTF_MIN];
	int  rv = stbsp_vsprintfcb(rb_putcb, tx, b, fmt, ap);
	va_end(ap);
	usart->CR1 |= USART1_CR1_TXEIE; // enable transmissions

	return rv;
}
