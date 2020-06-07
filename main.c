#include "STM32F411.h"

#include "stm32f411_irqn.h"
#include "gpio2.h"
#include "serial.h"

enum {
	LED0_PIN = PC13,
	USART1_TX_PIN = PA9, 
	//USART1_RX_PIN = PA10,
	// USART1_TX_PIN = PB6,
	// USART1_RX_PIN = PB7,
};

static struct gpio_config_t {
	enum GPIO_Pin pins;
	enum GPIO_Conf mode;
}  pin_cfgs[] = {
    {LED0_PIN, GPIO_ODO},
    {USART1_TX_PIN, GPIO_AF7_USART12|GPIO_HIGH},    
//	{USART1_RX_PIN, Mode_IPU},
    {0, 0}, // sentinel
};


static volatile uint64_t clockticks = STK_LOAD_RELOAD + 1; // rolls over after 2^64/96MHz = 6089.1097 years
void SysTick_Handler(void) { clockticks += STK_LOAD_RELOAD + 1; }
static inline uint64_t cycleCount(void) { return clockticks - (uint32_t)(STK.VAL & STK_LOAD_RELOAD); }

void delay(uint32_t usec) {
    uint64_t now = cycleCount();
    // then = now + usec * clockspeed_hz / (usec/sec)
    uint64_t then = now + 96 * usec;
    while (cycleCount() < then)
        __NOP(); // wait
}

static uint8_t tx1buf[256];


void main(void) {

	//uint8_t rf = RCC.CSR >> 24;

    RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	STK.LOAD = STK_LOAD_RELOAD; // max value
	STK.VAL  = 0;               // reset
	STK.CTRL = STK_CTRL_CLKSOURCE | STK_CTRL_TICKINT | STK_CTRL_ENABLE; // enable at AHB == 96MHz

    // NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    // for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
    //         NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    // }


	RCC.AHB1ENR |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	RCC.APB2ENR |= RCC_APB2ENR_USART1EN;
	serial_init(&USART1, 115200, tx1buf, sizeof(tx1buf));

	for (int i = 0; ; i++) {
		digitalToggle(LED0_PIN);

		delay(200*1000);
		serial_printf(&USART1, "%i\n", i);
	}
	
}