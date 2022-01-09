#pragma once

#include "stm32f411.h"

enum IRQn_Type {
	//  Cortex-M4 Processor Exceptions Numbers
	IRQ_None             = -16, // 0 position of estack reset pointer
	IRQ_Reset            = -15, // 1 Reset, not a real IRQ
	IRQ_NonMaskableInt   = -14, // 2 Non Maskable Interrupt
	IRQ_Reserved_3       = -13,
	IRQ_MemoryManagement = -12, // 4 Cortex-M4 Memory Management Interrupt
	IRQ_BusFault         = -11, // 5 Cortex-M4 Bus Fault Interrupt
	IRQ_UsageFault       = -10, // 6 Cortex-M4 Usage Fault Interrupt
	IRQ_Reserved_7       = -9,
	IRQ_Reserved_8       = -8,
	IRQ_Reserved_9       = -7,
	IRQ_Reserved_10      = -6,
	IRQ_SVCall           = -5, // 11 Cortex-M4 SV Call Interrupt
	IRQ_DebugMonitor     = -4, // 12 Cortex-M4 Debug Monitor Interrupt
	IRQ_Reserved_13      = -3,
	IRQ_PendSV           = -2, // 14 Cortex-M4 Pend SV Interrupt
	IRQ_SysTick          = -1, // 15 Cortex-M4 System Tick Interrupt

	//  Device specific Interrupt Numbers
	IRQ_WWDG               = 0,  // Window WatchDog Interrupt
	IRQ_PVD                = 1,  // PVD detection Interrupt
	IRQ_TAMP_STAMP         = 2,  // Tamper and TimeStamp interrupts
	IRQ_RTC_WKUP           = 3,  // RTC Wakeup interrupt
	IRQ_FLASH              = 4,  // FLASH global Interrupt
	IRQ_RCC                = 5,  // RCC global Interrupt
	IRQ_EXTI0              = 6,  // EXTI Line0 Interrupt
	IRQ_EXTI1              = 7,  // EXTI Line1 Interrupt
	IRQ_EXTI2              = 8,  // EXTI Line2 Interrupt
	IRQ_EXTI3              = 9,  // EXTI Line3 Interrupt
	IRQ_EXTI4              = 10, // EXTI Line4 Interrupt
	IRQ_DMA1_Stream0       = 11, // DMA1 Stream 0 global Interrupt
	IRQ_DMA1_Stream1       = 12, // DMA1 Stream 1 global Interrupt
	IRQ_DMA1_Stream2       = 13, // DMA1 Stream 2 global Interrupt
	IRQ_DMA1_Stream3       = 14, // DMA1 Stream 3 global Interrupt
	IRQ_DMA1_Stream4       = 15, // DMA1 Stream 4 global Interrupt
	IRQ_DMA1_Stream5       = 16, // DMA1 Stream 5 global Interrupt
	IRQ_DMA1_Stream6       = 17, // DMA1 Stream 6 global Interrupt
	IRQ_ADC                = 18, // ADC1, ADC2 and ADC3 global Interrupts
	IRQ_RESERVED_19        = 19,
	IRQ_RESERVED_20        = 20,
	IRQ_RESERVED_21        = 21,
	IRQ_RESERVED_22        = 22,
	IRQ_EXTI9_5            = 23, // External Line[9:5] Interrupts
	IRQ_TIM1_BRK_TIM9      = 24, // TIM1 Break interrupt and TIM9 global interrupt
	IRQ_TIM1_UP_TIM10      = 25, // TIM1 Update Interrupt and TIM10 global interrupt
	IRQ_TIM1_TRG_COM_TIM11 = 26, // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
	IRQ_TIM1_CC            = 27, // TIM1 Capture Compare Interrupt
	IRQ_TIM2               = 28, // TIM2 global Interrupt
	IRQ_TIM3               = 29, // TIM3 global Interrupt
	IRQ_TIM4               = 30, // TIM4 global Interrupt
	IRQ_I2C1_EV            = 31, // I2C1 Event Interrupt
	IRQ_I2C1_ER            = 32, // I2C1 Error Interrupt
	IRQ_I2C2_EV            = 33, // I2C2 Event Interrupt
	IRQ_I2C2_ER            = 34, // I2C2 Error Interrupt
	IRQ_SPI1               = 35, // SPI1 global Interrupt
	IRQ_SPI2               = 36, // SPI2 global Interrupt
	IRQ_USART1             = 37, // USART1 global Interrupt
	IRQ_USART2             = 38, // USART2 global Interrupt
	IRQ_RESERVED_39        = 39,
	IRQ_EXTI15_10          = 40, // External Line[15:10] Interrupts
	IRQ_RTC_Alarm          = 41, // RTC Alarm (A and B) Interrupt
	IRQ_OTG_FS_WKUP        = 42, // USB OTG FS Wakeup Unterrupt
	IRQ_RESERVED_43        = 43,
	IRQ_RESERVED_44        = 44,
	IRQ_RESERVED_45        = 45,
	IRQ_RESERVED_46        = 46,
	IRQ_DMA1_Stream7       = 47, // DMA1 Stream7 Interrupt
	IRQ_RESERVED_48        = 48,
	IRQ_SDIO               = 49, // SDIO global Interrupt
	IRQ_TIM5               = 50, // TIM5 global Interrupt
	IRQ_SPI3               = 51, // SPI3 global Interrupt
	IRQ_RESERVED_52        = 52,
	IRQ_RESERVED_53        = 53,
	IRQ_RESERVED_54        = 54,
	IRQ_RESERVED_55        = 55,
	IRQ_DMA2_Stream0       = 56, // DMA2 Stream 0 global Interrupt
	IRQ_DMA2_Stream1       = 57, // DMA2 Stream 1 global Interrupt
	IRQ_DMA2_Stream2       = 58, // DMA2 Stream 2 global Interrupt
	IRQ_DMA2_Stream3       = 59, // DMA2 Stream 3 global Interrupt
	IRQ_DMA2_Stream4       = 60, // DMA2 Stream 4 global Interrupt
	IRQ_RESERVED_61        = 61,
	IRQ_RESERVED_62        = 62,
	IRQ_RESERVED_63        = 63,
	IRQ_RESERVED_64        = 64,
	IRQ_RESERVED_65        = 65,
	IRQ_RESERVED_66        = 66,
	IRQ_OTG_FS             = 67, // USB OTG FS global Interrupt
	IRQ_DMA2_Stream5       = 68, // DMA2 Stream 5 global interrupt
	IRQ_DMA2_Stream6       = 69, // DMA2 Stream 6 global interrupt
	IRQ_DMA2_Stream7       = 70, // DMA2 Stream 7 global interrupt
	IRQ_USART6             = 71, // USART6 global interrupt
	IRQ_I2C3_EV            = 72, // I2C3 event interrupt
	IRQ_I2C3_ER            = 73, // I2C3 error interrupt
	IRQ_RESERVED_74        = 74,
	IRQ_RESERVED_75        = 75,
	IRQ_RESERVED_76        = 76,
	IRQ_RESERVED_77        = 77,
	IRQ_RESERVED_78        = 78,
	IRQ_RESERVED_79        = 79,
	IRQ_RESERVED_80        = 80,
	IRQ_FPU                = 81, // FPU global interrupt
	IRQ_RESERVED_82        = 82,
	IRQ_RESERVED_83        = 83,
	IRQ_SPI4               = 84, // SPI4 global Interrupt
	IRQ_SPI5               = 85, // SPI5 global Interrupt
};

inline void IRQ_Enable( enum IRQn_Type irqn) { (&NVIC.ISER0)[(((uint32_t)irqn) >> 5UL) & 0x3UL] = 1UL << ((uint32_t)irqn & 0x1FUL); }
inline void IRQ_Disable(enum IRQn_Type irqn) { (&NVIC.ICER0)[(((uint32_t)irqn) >> 5UL) & 0x3UL] = 1UL << ((uint32_t)irqn & 0x1FUL); }

