#include "STM32F411.h"

extern void main(void);                            // in main.c
extern void (*vector_table[])(void);               // in vector.c
extern char _sidata, _sdata, _edata, _sbss, _ebss; // provided by linker script

static inline void systemInit(void) {
	fpu_cpacr_cpacr_set_cp(&FPU_CPACR, 0xf); /* CP10/CP11 Full Access */
	/* Reset the RCC clock configuration to the default reset state(for debug purpose) */
	RCC.CR |= RCC_CR_HSION;
	RCC.CFGR = 0;
	RCC.CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON );
	RCC.PLLCFGR = 0x24003010;   /* Documented reset value */
	RCC.CR &= ~RCC_CR_HSEBYP;
	RCC.CIR = 0;  // disable all interrupts
}

enum { 
	HSE_RDY_TIMEOUT = 5000,
	HSI_VALUE = 16000000,
	HSE_VALUE = 25000000,
};


static int setSysClockTo100MHz(void) {

	RCC.CR |= RCC_CR_HSEON;

	for (int i = 0; i < HSE_RDY_TIMEOUT; i++) {
		if (RCC.CR & RCC_CR_HSERDY)
			break;
	}

	if ((RCC.CR & RCC_CR_HSERDY) == 0) {
		return 0; // HSE clock failed to become ready
	}

	// Power control: Scale 1 mode <= 100 MHz
	RCC.APB1ENR |= RCC_APB1ENR_PWREN;
	pwr_cr_set_vos(&PWR, 3); 

	rcc_cfgr_set_hpre(&RCC, 0);     // AHB HCLK = SYSCLK / 1     100MHz
	rcc_cfgr_set_ppre1(&RCC, 4);    // APB1 PCLK = AHB HCLK / 2   50MHz
	rcc_cfgr_set_ppre2(&RCC, 0);    // APB2 PCLK = AHB HCLK / 1  100MHz

    /* Configure the main PLL */
    RCC.PLLCFGR = RCC_PLLCFGR_PLLSRC;
	rcc_pllcfgr_set_pllm(&RCC, 8);
	rcc_pllcfgr_set_plln(&RCC, 400);
	rcc_pllcfgr_set_pllp(&RCC, (4/2)-1); // 0x01 -> 4
	rcc_pllcfgr_set_pllq(&RCC, 7);

    /* Enable the main PLL */
    RCC.CR |= RCC_CR_PLLON;

	// Wait till PLL is ready
	while ((RCC.CR & RCC_CR_PLLRDY) == 0)
		__NOP();

    FLASH.ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN;
    flash_acr_set_latency(&FLASH, 2);

	/* Select PLL as system clock source */
	rcc_cfgr_set_sw(&RCC, 2);

	// Wait till PLL is used as system clock source
	while (rcc_cfgr_get_sws(&RCC) != 2)
		__NOP();

	return 1;
}

uint32_t getSystemCoreClock(void) {
	uint32_t clk = HSI_VALUE;

	switch (rcc_cfgr_get_sws(&RCC))	{
    
	}

	static const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
	uint32_t shr = AHBPrescTable[rcc_cfgr_get_hpre(&RCC)];

	return clk >> shr;
}

void Reset_Handler(void) {
	char* src = &_sidata;
	char* dst = &_sdata;

	while (dst < &_edata)
		*dst++ = *src++;

	for (dst = &_sbss; dst < &_ebss; dst++)
		*dst = 0;

	SCB.VTOR = (uintptr_t)&vector_table; /* Vector Table Relocation in Internal FLASH. */

	systemInit();

	// TODO: as fallback; maybe try switch to HSI/2 * PLL16 = 64MHz
	// just keep trying
	while (!setSysClockTo100MHz()) {
		__NOP();
	}

	main();

	for (;;)
		__NOP(); // hang
}
