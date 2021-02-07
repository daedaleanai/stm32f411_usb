#include "STM32F411.h"

#include "stm32f411_irqn.h"
#include "gpio2.h"
#include "serial.h"

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 34.1

enum {
	LED0_PIN = PC13,
	// USART1_TX_PIN = PA9, 
	// USART1_RX_PIN = PA10,
	// USART1_TX_PIN = PB6,
	// USART1_RX_PIN = PB7,
	USART2_TX_PIN = PA2,
	USART2_RX_PIN = PA3,

    USB_DM_PIN   = PA11, // USB D-
    USB_DP_PIN   = PA12, // USB D+

};

static struct gpio_config_t {
	enum GPIO_Pin pins;
	enum GPIO_Conf mode;
}  pin_cfgs[] = {
    {LED0_PIN, GPIO_ODO},
    {USART2_TX_PIN, GPIO_AF7_USART12|GPIO_HIGH},    
     {USB_DM_PIN|USB_DP_PIN, GPIO_AFA_OTGFS},
//  {USART1_TX_PIN, GPIO_AF7_USART12|GPIO_HIGH},    
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

static uint8_t tx2buf[256];

extern void init_usb(void);

void main(void) {

	uint8_t rf = RCC.CSR >> 24;

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

 digitalHi(PC13);

	RCC.APB1ENR |= RCC_APB1ENR_USART2EN;
	serial_init(&USART2, 8*115200, tx2buf, sizeof tx2buf);

	serial_printf(&USART2, "SWREV:%s\n", __REVISION__);
    serial_printf(&USART2, "CPUID:%08lx\n", SCB.CPUID);
    serial_printf(&USART2, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    serial_printf(&USART2, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
                      rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    serial_wait(&USART2);

	RCC.AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	delay(10);
	init_usb();

	serial_printf(&USART2, "otg_fs mode: %s\n", OTG_FS_GLOBAL.GINTSTS & OTG_FS_GLOBAL_GINTSTS_CMOD ? "HOST" : "DEVICE");


	for (;;) {
		__WFI();
		delay(250 * 1000);

	} 
	
}