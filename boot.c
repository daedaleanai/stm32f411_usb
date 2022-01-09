#include "stm32f411.h"
#include "stm32f411_irqn.h"

extern void main(void);                            // in main.c
extern void (*vector_table[])(void);               // in vector.c
extern char _sidata, _sdata, _edata, _sbss, _ebss; // provided by linker script

static inline void systemInit(void) {
	fpu_cpacr_cpacr_set_cp(&FPU_CPACR, 0xf); /* CP10/CP11 Full Access */

	/* Reset the RCC clock configuration to the default reset state(for debug purpose) */
	RCC.CR |= RCC_CR_HSION;
	RCC.CFGR = 0;
	RCC.CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_HSEBYP);
	RCC.PLLCFGR = 0x24003010;   // Documented reset value
	RCC.CIR = 0;  // disable all rcc interrupts
}

enum { 
	HSE_RDY_TIMEOUT = 5000,
};

static int setSysClockTo96MHz(void) {

	RCC.CR |= RCC_CR_HSEON;

	for (int i = 0; i < HSE_RDY_TIMEOUT; i++)
		if (RCC.CR & RCC_CR_HSERDY)
			break;

	if ((RCC.CR & RCC_CR_HSERDY) == 0)
		return 0; // HSE clock failed to become ready

	// Power control: Scale 1 mode <= 100 MHz
	RCC.APB1ENR |= RCC_APB1ENR_PWREN;
	pwr_cr_set_vos(&PWR, 3); 

	rcc_cfgr_set_hpre(&RCC, 0);     // AHB HCLK = SYSCLK / 1     96MHz
	rcc_cfgr_set_ppre1(&RCC, 4);    // APB1 PCLK = AHB HCLK / 2  48MHz
	rcc_cfgr_set_ppre2(&RCC, 0);    // APB2 PCLK = AHB HCLK / 1  96MHz

    /* Configure the main PLL */
    RCC.PLLCFGR = RCC_PLLCFGR_PLLSRC; 	 // select HSE as source (25MHz)
	rcc_pllcfgr_set_pllm(&RCC, 25);      // 2..63       : vco_in = HSE / m   1..2MHz                25/25 = 1MHz
	rcc_pllcfgr_set_plln(&RCC, 384);     // 50...432    : vco_out = vco_in * n = 100...432MHz      (25/25) * (8*48) = 384MHz
	rcc_pllcfgr_set_pllp(&RCC, (4/2)-1); // 0,1,2,3 -> p=2,4,6,8  : sysclk = vco_out / p <= 100MHz (25/25) * (8*48) / 4 = 96MHz
	rcc_pllcfgr_set_pllq(&RCC, 8);       // 2..15                 : usbclk = vco_out / q = 48Mhz  (vco_out = 8*48 = 384)

    RCC.CR |= RCC_CR_PLLON;

	// Wait till PLL is ready
	while ((RCC.CR & RCC_CR_PLLRDY) == 0)
		__NOP();

    FLASH.ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN;
    flash_acr_set_latency(&FLASH, 2);

	rcc_cfgr_set_sw(&RCC, 2);	// Select PLL as system clock source

	// Wait till PLL is used as system clock source
	while (rcc_cfgr_get_sws(&RCC) != 2)
		__NOP();

	return 1;
}

#if 0
enum {
	HSI_VALUE = 16000000,
	HSE_VALUE = 25000000,
};

static uint32_t getSystemCoreClock(void) {
	uint32_t sysclk = 0;

	switch (rcc_cfgr_get_sws(&RCC))	{
    case 0:
    	sysclk = HSI_VALUE;
    	break;
    case 1:
    	sysclk = HSE_VALUE;
    	break;
    case 2:
    	sysclk = (RCC.PLLCFGR & RCC_PLLCFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
    	sysclk /= rcc_pllcfgr_get_pllm(&RCC);
    	sysclk *= rcc_pllcfgr_get_plln(&RCC);
    	sysclk /= (rcc_pllcfgr_get_pllp(&RCC) + 1) * 2;
    	break; 
	}
	 
	uint8_t pre = rcc_cfgr_get_hpre(&RCC); // 0..16
	if (pre & 0x8) {
		pre &= 0x7;
		sysclk >>= (0x98764321UL >> (4*pre)) & 0xF;
	}

	return sysclk;
}
#endif

void Reset_Handler(void) {
	char* src = &_sidata;
	char* dst = &_sdata;

	while (dst < &_edata)
		*dst++ = *src++;

	for (dst = &_sbss; dst < &_ebss; dst++)
		*dst = 0;

	SCB.VTOR = (uintptr_t)&vector_table; /* Vector Table Relocation in Internal FLASH. */

	systemInit();

	while (!setSysClockTo96MHz()) {
		__NOP();
	}

	main();

	for (;;)
		__NOP(); // hang
}
