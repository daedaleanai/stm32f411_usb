#include "STM32F411.h"

extern void _estack(void); // fake definition, will be filled in by linker script.

// Hang, let the watchdog reboot us.
// TODO(lvd): reset usart0 and report unexpected irq
void default_IRQ_Handler(void) {
    for (;;) {
            __WFE();
    }
}

// CM4 core fault handlers
void Reset_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void NonMaskableInt_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void MemoryManagement_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_8_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_9_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SVCall_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DebugMonitor_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_13_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));

// STM32F411 IRQ Handlers
void WWDG_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PVD_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TAMP_STAMP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_WKUP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void FLASH_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RCC_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI0_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream0_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream6_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void ADC_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_19_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_20_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_21_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_22_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI9_5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_BRK_TIM9_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_UP_TIM10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_TRG_COM_TIM11_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_CC_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_39_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI15_10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_Alarm_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void OTG_FS_WKUP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_43_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_44_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_45_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_46_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Stream7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_48_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SDIO_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_52_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_53_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_54_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_55_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream0_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_61_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_62_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_63_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_64_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_65_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_66_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void OTG_FS_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream6_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_Stream7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART6_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C3_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C3_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_74_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_75_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_76_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_77_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_78_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_79_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_80_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void FPU_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_82_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RESERVED_83_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));


__attribute__((section(".isr_vector"))) void (*vector_table[])(void) = {
    _estack,
    Reset_Handler,
    NonMaskableInt_Handler,
    Reserved_3_Handler,
    MemoryManagement_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    Reserved_7_Handler,
    Reserved_8_Handler,
    Reserved_9_Handler,
    Reserved_10_Handler,
    SVCall_Handler,
    DebugMonitor_Handler,
    Reserved_13_Handler,
    PendSV_Handler,
    SysTick_Handler,
	WWDG_Handler,
	PVD_Handler,
	TAMP_STAMP_Handler,
	RTC_WKUP_Handler,
	FLASH_Handler,
	RCC_Handler,
	EXTI0_Handler,
	EXTI1_Handler,
	EXTI2_Handler,
	EXTI3_Handler,
	EXTI4_Handler,
	DMA1_Stream0_Handler,
	DMA1_Stream1_Handler,
	DMA1_Stream2_Handler,
	DMA1_Stream3_Handler,
	DMA1_Stream4_Handler,
	DMA1_Stream5_Handler,
	DMA1_Stream6_Handler,
	ADC_Handler,
	RESERVED_19_Handler,
	RESERVED_20_Handler,
	RESERVED_21_Handler,
	RESERVED_22_Handler,
	EXTI9_5_Handler,
	TIM1_BRK_TIM9_Handler,
	TIM1_UP_TIM10_Handler,
	TIM1_TRG_COM_TIM11_Handler,
	TIM1_CC_Handler,
	TIM2_Handler,
	TIM3_Handler,
	TIM4_Handler,
	I2C1_EV_Handler,
	I2C1_ER_Handler,
	I2C2_EV_Handler,
	I2C2_ER_Handler,
	SPI1_Handler,
	SPI2_Handler,
	USART1_Handler,
	USART2_Handler,
	RESERVED_39_Handler,
	EXTI15_10_Handler,
	RTC_Alarm_Handler,
	OTG_FS_WKUP_Handler,
	RESERVED_43_Handler,
	RESERVED_44_Handler,
	RESERVED_45_Handler,
	RESERVED_46_Handler,
	DMA1_Stream7_Handler,
	RESERVED_48_Handler,
	SDIO_Handler,
	TIM5_Handler,
	SPI3_Handler,
	RESERVED_52_Handler,
	RESERVED_53_Handler,
	RESERVED_54_Handler,
	RESERVED_55_Handler,
	DMA2_Stream0_Handler,
	DMA2_Stream1_Handler,
	DMA2_Stream2_Handler,
	DMA2_Stream3_Handler,
	DMA2_Stream4_Handler,
	RESERVED_61_Handler,
	RESERVED_62_Handler,
	RESERVED_63_Handler,
	RESERVED_64_Handler,
	RESERVED_65_Handler,
	RESERVED_66_Handler,
	OTG_FS_Handler,
	DMA2_Stream5_Handler,
	DMA2_Stream6_Handler,
	DMA2_Stream7_Handler,
	USART6_Handler,
	I2C3_EV_Handler,
	I2C3_ER_Handler,
	RESERVED_74_Handler,
	RESERVED_75_Handler,
	RESERVED_76_Handler,
	RESERVED_77_Handler,
	RESERVED_78_Handler,
	RESERVED_79_Handler,
	RESERVED_80_Handler,
	FPU_Handler,
	RESERVED_82_Handler,
	RESERVED_83_Handler,
	SPI4_Handler,
	SPI5_Handler,
};