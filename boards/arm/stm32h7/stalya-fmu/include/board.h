/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_STALYA_FMU_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_STALYA_FMU_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Stalya-FMU  board provides the following clock sources:
 *
 *   Y3:  12.000 MHz crystal for HSE
 *   Y2:  32.768 KHz crystal for LSE
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 12 MHz XTAL
 *   LSE: 32.768 kHz XTAL
 */

#define STM32_BOARD_XTAL        12000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 12,000,000
 *
 * When STM32_HSE_FREQUENCY / PLLM <= 2MHz VCOL must be selected.
 * VCOH otherwise.
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 400 MHz
 */

#define STM32_BOARD_USEHSE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (12,000,000 / 3) * 200 = 800 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 800 MHz / 2   = 400 MHz
 *   PLL1Q = PLL1_VCO/4  = 800 MHz / 4   = 200 MHz
 *   PLL1R = PLL1_VCO/8  = 800 MHz / 8   = 100 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(3)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(200)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 3) * 200)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG    (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVP2EN         | \
                                 RCC_PLLCFGR_DIVQ2EN         | \
                                 RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(3)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(200)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(8)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(8)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(8)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 3) * 200)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)

/* PLL3 */

#define STM32_PLLCFG_PLL3CFG    (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL3RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVP3EN		     | \
								                 RCC_PLLCFGR_DIVQ3EN		     | \
								                 RCC_PLLCFGR_DIVR3EN)
#define STM32_PLLCFG_PLL3M      RCC_PLLCKSELR_DIVM3(3)
#define STM32_PLLCFG_PLL3N      RCC_PLL3DIVR_N3(192)
#define STM32_PLLCFG_PLL3P      RCC_PLL3DIVR_P3(8)
#define STM32_PLLCFG_PLL3Q      RCC_PLL3DIVR_Q3(16)
#define STM32_PLLCFG_PLL3R      RCC_PLL3DIVR_R3(12)

#define STM32_VCO3_FREQUENCY    ((STM32_HSE_FREQUENCY / 3) * 192)
#define STM32_PLL3P_FREQUENCY   (STM32_VCO3_FREQUENCY / 8)
#define STM32_PLL3Q_FREQUENCY   (STM32_VCO3_FREQUENCY / 16)
#define STM32_PLL3R_FREQUENCY   (STM32_VCO3_FREQUENCY / 12)

/* SYSCLK = PLL1P = 400 MHz
 * CPUCLK = SYSCLK / 1 = 400 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (200 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */

/* APB1 clock (PCLK1) is HCLK/4 (50 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd4       /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/4 (50 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd4       /* PCLK2 = HCLK / 4 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB3 clock (PCLK3) is HCLK/4 (50 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd4        /* PCLK3 = HCLK / 4 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB4 clock (PCLK4) is HCLK/4 (50 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd4       /* PCLK4 = HCLK / 4 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source - HSI (64MHz) */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI (64MHz) */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL2P (100MHz) */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2

/* SPI45 clock source - PLL2Q (100MHz) */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_PLL2

/* SPI6 clock source - PLL2Q (100MHz) */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PLL2

/* USB 1 and 2 clock source - PLL3Q (48MHz) */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_PLL3

/* ADC 1 2 3 clock source - PLL3R (64MHz) */

#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FDCAN 1 2 clock source - PLL2Q (100MHz) */

#define STM32_RCC_D2CCIP1R_FDCANSEL  RCC_D2CCIP1R_FDCANSEL_PLL2   /* FDCAN 1 2 clock source */

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 4

/* SDMMC definitions ********************************************************/

/* Init 400kHz, PLL1Q/(2*250) */

#define STM32_SDMMC_INIT_CLKDIV     (250 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* Just set these to 25 MHz for now,
 * PLL1Q/(2*4), for default speed 12.5MB/s
 */

#define STM32_SDMMC_MMCXFR_CLKDIV   (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

/* LED definitions **********************************************************/

/* The Stalya-FMU board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The Stalya FMU board supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/*
 * USART2 	ttyS0	  TEL1
 * USART3 	ttyS1	  GPS1
 * UART4 	  ttyS2	  TEL4
 * UART5 	  ttyS3	  GPS2
 * USART6 	ttyS4	  IOP1
 * UART7 	  ttyS5	  TEL2
 * UART8 	  ttyS6	  TEL3
*/

/* USART2 (Telemetry 1)*/

#define GPIO_USART2_RX    (GPIO_USART2_RX_1  | GPIO_SPEED_100MHz)   /* PA3 */
#define GPIO_USART2_TX    (GPIO_USART2_TX_1  | GPIO_SPEED_100MHz)   /* PA2 */
#define GPIO_USART2_RTS   (GPIO_USART2_RTS_1 | GPIO_SPEED_100MHz)   /* PA1 */
#define GPIO_USART2_CTS   (GPIO_USART2_CTS_NSS_1 | GPIO_SPEED_100MHz) /* PA0 */

/* USART3 (Main GPS) */

#define GPIO_USART3_RX    (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz)    /* PD9 */
#define GPIO_USART3_TX    (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz)    /* PD8 */

/* USART6 (I/O Processor) */

#define GPIO_USART6_RX    (GPIO_USART6_RX_1 | GPIO_SPEED_100MHz)    /* PC7 */
#define GPIO_USART6_TX    (GPIO_USART6_TX_1 | GPIO_SPEED_100MHz)    /* PC6 */

/* UART4 (Telemetry 4) */

#define GPIO_UART4_RX     (GPIO_UART4_RX_4 | GPIO_SPEED_100MHz)    /* PC11 */
#define GPIO_UART4_TX     (GPIO_UART4_TX_4 | GPIO_SPEED_100MHz)    /* PC10 */

/* UART5 (Secondary GPS) */

#define GPIO_UART5_RX     (GPIO_UART5_RX_1 | GPIO_SPEED_100MHz)    /* PB12 */
#define GPIO_UART5_TX     (GPIO_UART5_TX_1 | GPIO_SPEED_100MHz)    /* PB13 */

/* UART7 (Telemetry 2) */

#define GPIO_UART7_RX     (GPIO_UART7_RX_3 | GPIO_SPEED_100MHz)    /* PE7  */
#define GPIO_UART7_TX     (GPIO_UART7_TX_3 | GPIO_SPEED_100MHz)    /* PE8  */
#define GPIO_UART7_RTS    (GPIO_UART7_RTS_1 | GPIO_SPEED_100MHz)   /* PE9  */
#define GPIO_UART7_CTS    (GPIO_UART7_CTS_1 | GPIO_SPEED_100MHz)   /* PE10 */

/* UART8 (Telemetry 3) */

#define GPIO_UART8_RX     (GPIO_UART8_RX_1 | GPIO_SPEED_100MHz)    /* PE0 */
#define GPIO_UART8_TX     (GPIO_UART8_TX_1 | GPIO_SPEED_100MHz)    /* PE1 */

/* I2C2 (Internal Control Bus) */

#define GPIO_I2C2_SCL     (GPIO_I2C2_SCL_1  | GPIO_SPEED_50MHz)     /* PB10 */
#define GPIO_I2C2_SDA     (GPIO_I2C2_SDA_1  | GPIO_SPEED_50MHz)     /* PB11 */

/* I2C4 (Secondary IMU Bus) */

#define GPIO_I2C4_SCL     (GPIO_I2C4_SCL_5 | GPIO_SPEED_50MHz)      /* PB8 */
#define GPIO_I2C4_SDA     (GPIO_I2C4_SDA_5 | GPIO_SPEED_50MHz)      /* PB9 */

/* SPI1 (Primary IMU Bus) */

#define GPIO_SPI1_MISO    (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)     /* PA6 */
#define GPIO_SPI1_MOSI    (GPIO_SPI1_MOSI_1 | GPIO_SPEED_50MHz)     /* PA7 */
#define GPIO_SPI1_SCK     (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz)     /* PA5 */

/* SPI4 (FRAM) */

#define GPIO_SPI4_MISO    (GPIO_SPI4_MISO_2 | GPIO_SPEED_50MHz)     /* PE5 */
#define GPIO_SPI4_MOSI    (GPIO_SPI4_MOSI_2 | GPIO_SPEED_50MHz)     /* PE6 */
#define GPIO_SPI4_SCK     (GPIO_SPI4_SCK_2  | GPIO_SPEED_50MHz)     /* PE2 */

/* TIM1 (PWM Inputs) */

#define GPIO_TIM1_CH1IN   (GPIO_TIM1_CH1IN_1  | GPIO_SPEED_50MHz)   /* PA8  */
#define GPIO_TIM1_CH2IN   (GPIO_TIM1_CH2IN_1  | GPIO_SPEED_50MHz)   /* PA9  */
#define GPIO_TIM1_CH3IN   (GPIO_TIM1_CH3IN_1  | GPIO_SPEED_50MHz)   /* PA10 */

/* TIM4 (PWM Outputs) */

#define GPIO_TIM4_CH1OUT  (GPIO_TIM4_CH1OUT_2  | GPIO_SPEED_50MHz)  /* PD12 */
#define GPIO_TIM4_CH2OUT  (GPIO_TIM4_CH2OUT_2  | GPIO_SPEED_50MHz)  /* PD13 */
#define GPIO_TIM4_CH3OUT  (GPIO_TIM4_CH3OUT_2  | GPIO_SPEED_50MHz)  /* PD14 */
#define GPIO_TIM4_CH4OUT  (GPIO_TIM4_CH4OUT_2  | GPIO_SPEED_50MHz)  /* PD15 */

/* OTGFS */

#define GPIO_OTGFS_DM     (GPIO_OTGFS_DM_0  | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP     (GPIO_OTGFS_DP_0  | GPIO_SPEED_100MHz)

/* SDMMC1 */

#define GPIO_SDMMC1_CK    (GPIO_SDMMC1_CK_0  | GPIO_SPEED_50MHz)    /* PC12 */
#define GPIO_SDMMC1_CMD   (GPIO_SDMMC1_CMD_0 | GPIO_SPEED_50MHz)    /* PD2  */
#define GPIO_SDMMC1_D0    (GPIO_SDMMC1_D0_0  | GPIO_SPEED_50MHz)    /* PC8  */

/* CAN Bus  */

#define GPIO_CAN1_TX      (GPIO_CAN1_TX_3 | GPIO_SPEED_50MHz)       /* PD1  */
#define GPIO_CAN1_RX      (GPIO_CAN1_RX_3 | GPIO_SPEED_50MHz)       /* PD0  */

#define GPIO_CAN2_TX      (GPIO_CAN2_TX_2 | GPIO_SPEED_50MHz)       /* PB6  */
#define GPIO_CAN2_RX      (GPIO_CAN2_RX_2 | GPIO_SPEED_50MHz)       /* PB5  */

/* DMA **********************************************************************/

#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0

#define DMAMAP_UART4_RX   DMAMAP_DMA12_UART4RX_0
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_0

#define DMAMAP_UART5_RX   DMAMAP_DMA12_UART5RX_0
#define DMAMAP_UART5_TX   DMAMAP_DMA12_UART5TX_0

#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_1
#define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_1

#define DMAMAP_USART3_RX  DMAMAP_DMA12_USART3RX_1
#define DMAMAP_USART3_TX  DMAMAP_DMA12_USART3TX_1

#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_1
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_1

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_STALYA_FMU_INCLUDE_BOARD_H */
