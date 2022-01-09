#include "gpio2.h"


static uint32_t maskn(uint16_t pins, uint8_t bits, uint32_t val) {
	uint32_t mask = 0;
	while (pins) {
		if (pins & 1) mask |= val;
		pins >>= 1;
		val <<= bits;
	}
	return mask;
}



void gpioConfig(enum GPIO_Pin pins, enum GPIO_Conf conf) {
	//assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios

	struct GPIOA_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;

	pins &= Pin_All;

	if ((conf & 0x3000) == GPIO_ANALOG) {
		conf = GPIO_ANALOG; // clear all other flags
	}

	if ((conf & 0x0030) == 0x0030) {
		conf &= ~0x0030; // both PU and PD -> both off (reserved according to manual)
	}

	uint32_t mode = (conf & 0x3000) >> 12;
	uint32_t af   = (conf & 0x0f00) >> 8;
	uint32_t od   = (conf & 0x0080) >> 7;
	uint32_t pupd = (conf & 0x0030) >> 4;
	uint32_t spd  = (conf & 0x0003);

	uint32_t msk2 = maskn(pins, 2, 0x3);
	gpio->MODER   &= ~msk2;
	gpio->PUPDR   &= ~msk2;
	gpio->OSPEEDR &= ~msk2;

	if (od) {
		gpio->OTYPER |= pins;
	} else {
		gpio->OTYPER &= ~pins;		
	}

	gpio->OSPEEDR |= maskn(pins, 2, spd);
	gpio->PUPDR   |= maskn(pins, 2, pupd);
	gpio->MODER   |= maskn(pins, 2, mode);

	if (mode == 2) { // AF OUTPUT
		gpio->AFRL &= ~maskn(pins & 0xff, 4, 0xf);
		gpio->AFRL |= maskn(pins & 0xff, 4, af);
		gpio->AFRH &= ~maskn((pins>>8) & 0xff, 4, 0xf);
		gpio->AFRH |= maskn((pins>>8) & 0xff, 4, af);
	}
}

uint32_t gpioLock(enum GPIO_Pin pins) {
	//assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios

	struct GPIOA_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;

	pins &= Pin_All;
	gpio->LCKR = pins | (1<<16);	
	gpio->LCKR = pins ;	
	gpio->LCKR = pins | (1<<16);	
	return gpio->LCKR;
}
