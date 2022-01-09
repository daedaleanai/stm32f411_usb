#include "usart.h"
extern inline void usart_wait(struct USART1_Type* usart);

void usart_init(struct USART1_Type* usart, int baud) {
	usart->CR1 = 0;
	usart->CR2 = 0;
	usart->CR3 = 0;

	// PCLK2 for USART1,6, PCLK1 for USART 2, 
    // this depends on SetSystemclockTo96MHz  
    // APB1 PCLK = AHB HCLK / 2 = 48MHz 
    // APB2 PCLK = AHB HCLK / 1 = 96MHz
    uint32_t clk = 96000000;
    if (usart == &USART2) {
    	clk /= 2;
    }
	usart->BRR = clk / baud;
	usart->CR1 |= USART1_CR1_UE | USART1_CR1_TE;
}

void usart_irq_handler(struct USART1_Type* usart, struct Ringbuffer* rb) {
	if (!ringbuffer_empty(rb)) {
		if ((usart->SR & USART1_SR_TXE) != 0) {
			usart->DR = ringbuffer_get_tail(rb);   
		}
	} else {
		usart->CR1 &= ~USART1_CR1_TXEIE;
	}
	return;
}

size_t usart_puts(struct USART1_Type* usart, struct Ringbuffer* rb, const char *buf, size_t len) {
    size_t r = ringbuffer_puts(rb, buf, len);
    // on overflow zap the buffer and leave a marker for the user that data was lost
    if (r < len) {
        ringbuffer_clear(rb);
        ringbuffer_puts(rb, "!OVFL!", 6);
    }
	usart->CR1 |= USART1_CR1_TXEIE; // enable transmissions
    return r;
}
