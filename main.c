#include "STM32F411.h"

#include "gpio2.h"

enum {
	LED0_PIN = PC13,
};

static struct gpio_config_t {
	enum GPIO_Pin pins;
	enum GPIO_Conf mode;
}  pin_cfgs[] = {
    {LED0_PIN, GPIO_ODO},
    {0, 0}, // sentinel
};


void main(void) {

	RCC.AHB1ENR |=	RCC_AHB1ENR_GPIOCEN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	for (;;) {
		digitalToggle(LED0_PIN);

		for (int i = 0; i < 10000000; i++) 
			__NOP();

	}
	
}