#include "gpio2.h"
#include "printf.h"
#include "stm32f411.h"
#include "stm32f411_irqn.h"
#include "usart.h"
#include "usb.h"

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 34.1

#define	LED0_PIN       PC13
#define	USART2_TX_PIN  PA2
#define	USART2_RX_PIN  PA3
#define	USB_DM_PIN     PA11 // USB D-
#define	USB_DP_PIN     PA12 // USB D+

/* clang-format off */
static struct gpio_config_t {
	enum GPIO_Pin pins;
	enum GPIO_Conf mode;
}  pin_cfgs[] = {
    {LED0_PIN, GPIO_ODO},
    {USART2_TX_PIN, GPIO_AF7_USART12|GPIO_HIGH},    
    {USB_DM_PIN|USB_DP_PIN, GPIO_AFA_OTGFS},
    {0, 0}, // sentinel
};

// // grouping 5: prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
// enum { IRQ_PRIORITY_GROUPING = 5 };
// static struct {
//     enum IRQn_Type irq;
//     uint8_t        group, sub;
// } irqprios[] = {
//     {IRQ_SysTick, 0, 0},
//     {IRQ_OTG_FS,  1, 0},
//     {IRQ_USART2,  2, 0},
//     {IRQ_TIM3,    3, 0},
//     {IRQ_None, 0xff, 0xff},
// };
/* clang-format on */

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

static volatile uint64_t clockticks = STK_LOAD_RELOAD + 1; // rolls over after 2^64/96MHz = 6089.1097 years
void                     SysTick_Handler(void) { clockticks += STK_LOAD_RELOAD + 1; }
static inline uint64_t   cycleCount(void) { return clockticks - (uint32_t)(STK.VAL & STK_LOAD_RELOAD); }

// static void delay(uint32_t usec) {
//     uint64_t now = cycleCount();
//     // then = now + usec * clockspeed_hz / (usec/sec)
//     uint64_t then = now + 96 * usec;
//     while (cycleCount() < then)
//         __NOP(); // wait
// }

static struct Ringbuffer usart2tx;

void          USART2_Handler(void) { usart_irq_handler(&USART2, &usart2tx); }
static size_t u2puts(const char* buf, size_t len) { return usart_puts(&USART2, &usart2tx, buf, len); }
static size_t usb_puts(const char* buf, size_t len) { return usb_send(buf, len); }

void OTG_FS_Handler(void) {
	uint64_t now = cycleCount();
	led0_toggle();
	static int i = 0;
	cbprintf(u2puts, "%lld IRQ %i: %s\n", now / 96, i++, usb_state_str(usb_state()));
	uint8_t buf[64];
	size_t  len = usb_recv(buf, sizeof buf);
	if (len > 0) {
		cbprintf(u2puts, "received %i: %*s\n", len, len, buf);
	}
}

void TIM3_Handler(void) {
	if ((TIM3.SR & TIM3_SR_UIF) == 0)
		return;
	TIM3.SR &= ~TIM3_SR_UIF;
	static int i = 0;
	cbprintf(u2puts, "USB %i: %s\n", i, usb_state_str(usb_state()));
	cbprintf(usb_puts, "bingo %i\n", i++);
}

void main(void) {

	uint8_t rf = RCC.CSR >> 24;

	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	STK.LOAD = STK_LOAD_RELOAD;                                         // max value
	STK.VAL  = 0;                                                       // reset
	STK.CTRL = STK_CTRL_CLKSOURCE | STK_CTRL_TICKINT | STK_CTRL_ENABLE; // enable at AHB == 96MHz

	// NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
	// for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
	//         NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
	// }

	RCC.AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	RCC.APB1ENR |= RCC_APB1ENR_TIM3EN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PCAll);

	led0_on();

	RCC.APB1ENR |= RCC_APB1ENR_USART2EN;
	usart_init(&USART2, 921600);
	IRQ_Enable(IRQ_USART2);

	cbprintf(u2puts, "SWREV:%s\n", __REVISION__);
	cbprintf(u2puts, "CPUID:%08lx\n", SCB.CPUID);
	cbprintf(u2puts, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	cbprintf(u2puts, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
	         rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
	usart_wait(&USART2);

	// enable 1Hz TIM3
	TIM3.DIER |= TIM3_DIER_UIE;
	TIM3.PSC = 7200 - 1;  // 72MHz/7200   = 10KHz
	TIM3.ARR = 10000 - 1; // 10KHz/10000  = 1Hz
	TIM3.CR1 |= TIM3_CR1_CEN;
	IRQ_Enable(IRQ_TIM3);

	RCC.AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	usb_init();
	IRQ_Enable(IRQ_OTG_FS);

	for (;;)
		__WFI();
}