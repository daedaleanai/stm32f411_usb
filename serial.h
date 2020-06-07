#pragma once

#include "stdarg.h"
#include "stddef.h"

#include "STM32F411.h"

// serial_init() initializes USART{1,2,6}, to use buf[:bufsz].  bufsz must be a power of 2;
//
// The bytes will transmitted be as 8 data bits, no Parity bit, 1 stop bit (8N1) at the specified baud rate.
//
// Before calling serial_init, make sure to set up the GPIO pins: TX to AF_PP/10MHz. RX to IN FLOATING or Pull-up.
// and to enable the USART in the RCC register:	RCC->APBxENR |= RCC_APBxENR_USARTyEN;
void serial_init(struct USART1_Type* usart, int baud, uint8_t* buf, uint16_t bufsz);

// serial_printf() interprets fmt as a format string for the variable parameters and copies a corresponding
// message to the usarts ringbuffer for transmission.  if the buffer is full, zaps the buffer and prints "!OVFL!".
//
int serial_printf(struct USART1_Type* usart, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

void serial_wait(struct USART1_Type* usart);