#pragma once

#include "stdarg.h"
#include "stddef.h"

#include "stm32f411.h"
#include "printf.h"

// usart_init() initializes USART{1,2,3}.
//
// The bytes will transmitted be as 8 data bits, no Parity bit, 1 stop bit (8N1) at the specified baud rate.
//
// Before calling usart_init, make sure to set up the GPIO pins: TX to AF_PP/10MHz. RX to IN FLOATING or Pull-up.
// and to enable the USART in the RCC register:	RCC->APBxENR |= RCC_APBxENR_USARTyEN;
void usart_init(struct USART1_Type* usart, int baud);

// Create IRQ Handlers for the usarts you use like so:
// 	void USARTx_IRQ_Handler(void) { usart_irq_andler(&USARTx, txbufx); }
void usart_irq_handler(struct USART1_Type* usart, struct Ringbuffer* rb);

// create a puts for use with cbprintf() like so:
// 	size_t ux_puts(const char *buf, size_t len) { return usart_puts(&USARTx, tx_bufx, buf, len); }
size_t usart_puts(struct USART1_Type* usart, struct Ringbuffer* rb, const char *buf, size_t len);

inline void usart_wait(struct USART1_Type* usart) {
	while (usart->CR1 & USART1_CR1_TXEIE)
		;
}
