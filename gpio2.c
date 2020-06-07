#include "gpio2.h"

void gpioConfig(enum GPIO_Pin pins, enum GPIO_Conf mode) {
	assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios

	struct GPIO_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;
	uint32_t pos;
	for (pos = 0; pos < 16; ++pos) {
		if ((pins & (0x1 << pos)) == 0)
			continue;

		// MODE

		// TYPE

		// PUPD

		// OSPEED

	}
}

uint32_t gpioLock(enum GPIO_Pin pins) {
	assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios

	struct GPIO_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;

	pins &= Pin_All;
	gpio->LCKR = pins | (1<<16);	
	gpio->LCKR = pins ;	
	gpio->LCKR = pins | (1<<16);	
	return gpio->LCKR;
}
