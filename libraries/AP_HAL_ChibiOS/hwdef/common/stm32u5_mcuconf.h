/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  STM32U5 mcuconf. The U5 port uses the ChibiOS clock-tree settings
  (STM32_CFG_*): the _VALUE keys are divider and multiplier values, see the
  port's generated clocktree.h.
 */
#pragma once

#ifndef STM32_LSECLK
#define STM32_LSECLK 32768U
#endif
#ifndef STM32_LSEDRV
#define STM32_LSEDRV (3U << 3U)
#endif

#ifndef MCUCONF_H
#define MCUCONF_H

#define STM32U5xx_MCUCONF
#define STM32U575_MCUCONF
#define STM32U585_MCUCONF

#define STM32_NO_INIT                       FALSE
#define STM32_CFG_CLOCK_DYNAMIC             FALSE

/*
 * Clock tree: SYSCLK = PLL1R = 160 MHz, voltage range 1. The port enables
 * the EPOD booster above 55 MHz by itself.
 */
#if STM32_HSECLK == 16000000U
// 16 MHz crystal: M=1 N=20 R=2, VCO 320 MHz
#define STM32_CFG_HSE_ENABLE                TRUE
#define STM32_CFG_PLL1IN_SEL                RCC_PLL1CFGR_PLL1SRC_HSE
#define STM32_CFG_PLL1REF_VALUE             1
#define STM32_CFG_PLL1VCO_VALUE             20
#elif STM32_HSECLK == 0U
// no crystal: MSIS 4 MHz, M=1 N=80 R=2, VCO 320 MHz
#define STM32_CFG_HSE_ENABLE                FALSE
#define STM32_CFG_PLL1IN_SEL                RCC_PLL1CFGR_PLL1SRC_MSIS
#define STM32_CFG_PLL1REF_VALUE             1
#define STM32_CFG_PLL1VCO_VALUE             80
#else
#error "Unsupported HSE clock"
#endif
#define STM32_CFG_PLL1R_VALUE               2
#define STM32_CFG_SYSCLK_SEL                RCC_CFGR1_SW_PLL1R
#define STM32_CFG_PWR_VOSR                  PWR_VOSR_VOS_RANGE1
#define STM32_CFG_HSI16_ENABLE              FALSE
#define STM32_CFG_HSI48_ENABLE              FALSE
#define STM32_CFG_LSI_ENABLE                FALSE
#define STM32_CFG_LSE_ENABLE                FALSE
#define STM32_CFG_LPUART1_SEL               RCC_CCIPR3_LPUART1SEL_PCLK3

/*
 * PG2-PG15 run from VDDIO2 and stay isolated until it is declared valid.
 * Boards without anything on port G can override this with 0.
 */
#ifndef STM32_PWR_SVMCR
#define STM32_PWR_SVMCR                     PWR_SVMCR_IO2SV
#endif

/*
 * Instruction cache, 2-way. Code runs from flash with 4 wait states at
 * 160 MHz; without the cache the bootloader's CRC of 2 MB takes 2.5 s
 * (uploader.py gives up after 2 s). flash.c invalidates it after erase
 * and program operations.
 */
#ifndef STM32_ICACHE_CR
#define STM32_ICACHE_CR                     (ICACHE_CR_WAYSEL | ICACHE_CR_EN)
#endif

/*
 * IRQ priorities. The platform's stm32_isr.c serves every peripheral below,
 * so each one needs a priority even when its driver is off.
 */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_PRIORITY            6
#define STM32_IRQ_EXTI6_PRIORITY            6
#define STM32_IRQ_EXTI7_PRIORITY            6
#define STM32_IRQ_EXTI8_PRIORITY            6
#define STM32_IRQ_EXTI9_PRIORITY            6
#define STM32_IRQ_EXTI10_PRIORITY           6
#define STM32_IRQ_EXTI11_PRIORITY           6
#define STM32_IRQ_EXTI12_PRIORITY           6
#define STM32_IRQ_EXTI13_PRIORITY           6
#define STM32_IRQ_EXTI14_PRIORITY           6
#define STM32_IRQ_EXTI15_PRIORITY           6

#define STM32_IRQ_DAC1_PRIORITY             9
#define STM32_IRQ_FDCAN1_PRIORITY           10

#define STM32_IRQ_I2C1_PRIORITY             5
#define STM32_IRQ_I2C2_PRIORITY             5
#define STM32_IRQ_I2C3_PRIORITY             5
#define STM32_IRQ_I2C4_PRIORITY             5

#define STM32_IRQ_SPI1_PRIORITY             10
#define STM32_IRQ_SPI2_PRIORITY             10
#define STM32_IRQ_SPI3_PRIORITY             10

#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_UP_PRIORITY          7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM15_PRIORITY            7
#define STM32_IRQ_TIM16_PRIORITY            7
#define STM32_IRQ_TIM17_PRIORITY            7

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_LPUART1_PRIORITY          12

/*
 * Driver defaults; hwdef.h turns on what the board uses.
 */
#ifndef STM32_DAC_USE_DAC1_CH1
#define STM32_DAC_USE_DAC1_CH1              FALSE
#endif
#ifndef STM32_DAC_USE_DAC1_CH2
#define STM32_DAC_USE_DAC1_CH2              FALSE
#endif

#ifndef STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1                  FALSE
#endif
#ifndef STM32_I2C_USE_I2C2
#define STM32_I2C_USE_I2C2                  FALSE
#endif
#ifndef STM32_I2C_USE_I2C3
#define STM32_I2C_USE_I2C3                  FALSE
#endif
#ifndef STM32_I2C_USE_I2C4
#define STM32_I2C_USE_I2C4                  FALSE
#endif
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      STM32_DMA_ERROR_HOOK(i2cp)
#define STM32_I2C_ISR_LIMIT                 6
// interrupt-driven I2C until ArduPilot has a GPDMA backend
#ifndef STM32_I2C_USE_DMA
#define STM32_I2C_USE_DMA                   FALSE
#endif

#ifndef STM32_SPI_USE_SPI1
#define STM32_SPI_USE_SPI1                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI2
#define STM32_SPI_USE_SPI2                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI3
#define STM32_SPI_USE_SPI3                  FALSE
#endif
#define STM32_SPI_DMA_ERROR_HOOK(spip)      STM32_DMA_ERROR_HOOK(spip)

// ArduPilot's own DMA users (UARTDriver) allocate GPDMA channels through the ChibiOS DMA3 driver
#if defined(STM32_DMA_REQUIRED) && !defined(STM32_DMA3_REQUIRED)
#define STM32_DMA3_REQUIRED
#endif

#define STM32_ST_IRQ_PRIORITY               8
#ifndef STM32_ST_USE_TIMER
#define STM32_ST_USE_TIMER                  2
#endif

#ifndef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1           FALSE
#endif
#ifndef STM32_SERIAL_USE_USART2
#define STM32_SERIAL_USE_USART2           FALSE
#endif
#ifndef STM32_SERIAL_USE_USART3
#define STM32_SERIAL_USE_USART3           FALSE
#endif
#ifndef STM32_SERIAL_USE_UART4
#define STM32_SERIAL_USE_UART4            FALSE
#endif
#ifndef STM32_SERIAL_USE_UART5
#define STM32_SERIAL_USE_UART5            FALSE
#endif
#ifndef STM32_SERIAL_USE_LPUART1
#define STM32_SERIAL_USE_LPUART1          FALSE
#endif

#define STM32_USB_USE_USB1                  FALSE
#define STM32_WDG_USE_IWDG                  FALSE

#endif /* MCUCONF_H */
