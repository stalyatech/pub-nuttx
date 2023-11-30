/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/include/board.h
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

#ifndef __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* No not include STM32F7 header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking */

/* The STM32F7 Discovery board provides the following clock sources:
 *
 *   X1:  12 MHz oscillator for STM32F745VGT6 microcontroller and
 *        Ethernet PHY.
 *   X2:  32.768 KHz crystal for STM32F745VGT6 embedded RTC
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: On-board crystal frequency is 12MHz
 *   LSE: 32.768 kHz
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
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * USB OTG FS, SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The USB OTG FS requires a 48 MHz clock to work correctly. The SDMMC
 *   and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 */


/* Highest SYSCLK
 *
 * PLL_VCO = (12,000,000 / 6) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(6)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 6) * 216)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 9)

/* Configure factors for  PLLSAI clock */

#define CONFIG_STM32F7_PLLSAI           1
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(8)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0

/* Configure factors for  PLLI2S clock */

#define CONFIG_STM32F7_PLLI2S          1
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_USART3SRC  RCC_DCKCFGR2_USART3SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB

#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI

#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLL

#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY

/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

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

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* SDMMC */

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define STM32_SDMMC_INIT_CLKDIV      (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV  (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV  (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV   (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV   (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

/* LED definitions **********************************************************/

/* The ArduSimple-SBC board has 3 LEDs.
 * 
 * All of them can be controlled by software.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * 
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        0
#define BOARD_LED3        0
#define BOARD_NLEDS       3

#define BOARD_LD1         BOARD_LED1
#define BOARD_LD2         BOARD_LED2
#define BOARD_LD3         BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode
 * OS-relatedevents as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 * ----------------------  --------------------------  ------ ------ ----
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

/* Button definitions *******************************************************/

#define BUTTON_USER        0
#define NUM_BUTTONS        0
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 */

#define ADC1_DMA_CHAN     DMAMAP_ADC1_1       /* DMA2: STREAM0, CHAN0 */
#define DMAMAP_SDMMC1     DMAMAP_SDMMC1_1     /* DMA2: STREAM3, CHAN4 */
#define DMAMAP_USART1_RX  DMAMAP_USART1_RX_1  /* DMA2: STREAM2, CHAN4 */
#define DMAMAP_USART3_TX  DMAMAP_USART3_TX_1  /* DMA1: STREAM3, CHAN4 */
#define DMAMAP_USART6_RX  DMAMAP_USART6_RX_1  /* DMA2: STREAM1, CHAN5 */
#define DMAMAP_USART6_TX  DMAMAP_USART6_TX_1  /* DMA2: STREAM6, CHAN5 */
#define DMAMAP_SPI3_RX    DMAMAP_SPI3_RX_2    /* DMA1: STREAM2, CHAN0 */
#define DMAMAP_SPI3_TX    DMAMAP_SPI3_TX_2    /* DMA1: STREAM7, CHAN0 */

/* Alternate function pin selections ****************************************/

/* ADC1 */

#define GPIO_ADC1_IN0   GPIO_ADC1_IN0_0   /* PA0 */
#define GPIO_ADC1_IN4   GPIO_ADC1_IN4_0   /* PA4 */
#define GPIO_ADC1_IN6   GPIO_ADC1_IN6_0   /* PA6 */
#define GPIO_ADC1_IN10  GPIO_ADC1_IN10_0  /* PC0 */
#define GPIO_ADC1_IN12  GPIO_ADC1_IN12_0  /* PC2 */
#define GPIO_ADC1_IN13  GPIO_ADC1_IN13_0  /* PC3 */

/* TIM */

/* PWM
 * Use Timer 4
 */

#define GPIO_TIM4_CH1OUT (GPIO_TIM4_CH1OUT_2|GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH2OUT (GPIO_TIM4_CH2OUT_2|GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH3OUT (GPIO_TIM4_CH3OUT_2|GPIO_SPEED_50MHz)
#define GPIO_TIM4_CH4OUT (GPIO_TIM4_CH4OUT_2|GPIO_SPEED_50MHz)

/* USART1:
 *
 * USART1 is connected to the "FP9 GPS2" lines.
 *
 *   --------  ---------  -----
 *              STM32F7
 *   SIGNAME   FUNCTION   GPIO
 *   --------  ---------  -----
 *   GPS2_TX   USART1_TX  PB6
 *   GPS2_RX   USART1_RX  PB7
 *   --------  ---------  -----
 */

#define GPIO_USART1_TX (GPIO_USART1_TX_2|GPIO_SPEED_100MHz)
#define GPIO_USART1_RX (GPIO_USART1_RX_2|GPIO_SPEED_100MHz)

/* USART2:
 *
 * USART2 is connected to the "F9P GPS3" lines.
 *
 *   --------  ---------  -----
 *              STM32F7
 *   SIGNAME   FUNCTION   GPIO
 *   --------  ---------  -----
 *   GPS3_TX   USART2_TX  PD5
 *   GPS3_RX   USART2_RX  PD6
 *   --------  ---------  -----
 */

#define GPIO_USART2_TX (GPIO_USART2_TX_2|GPIO_SPEED_100MHz)
#define GPIO_USART2_RX (GPIO_USART2_RX_2|GPIO_SPEED_100MHz)

/* USART3:
 *
 * USART3 is connected to the "RS232" lines.
 *
 *   --------  ---------  -----
*               STM32F7
 *   SIGNAME   FUNCTION   GPIO
 *   --------  ---------  -----
 *   RS232_TX  USART2_TX  PD8
 *   RS232_RX  USART2_RX  PD9
 *   --------  ---------  -----
 */

#define GPIO_USART3_TX (GPIO_USART3_TX_3|GPIO_SPEED_100MHz)
#define GPIO_USART3_RX (GPIO_USART3_RX_3|GPIO_SPEED_100MHz)

/* USART6:
 *
 * USART6 is connected to the "XBee Socket B" lines.
 *
 *   --------  ---------  -----
 *              STM32F7
 *   SIGNAME   FUNCTION   GPIO
 *   --------  ---------  -----
 *   XBB_TX    USART6_TX  PC6
 *   XBB_RX    USART6_RX  PC7
 *   --------  ---------  -----
 */

#define GPIO_USART6_TX (GPIO_USART6_TX_1|GPIO_SPEED_100MHz)
#define GPIO_USART6_RX (GPIO_USART6_RX_1|GPIO_SPEED_100MHz)

/* UART4:
 *
 * UART4 is connected to the "XBEE Socket A" lines.
 *
 *   -------  ---------  -----
 *             STM32F7
 *   SIGNAME  FUNCTION   GPIO
 *   -------  ---------  -----
 *   XBA_TX   UART4_TX   PC10
 *   XBA_RX   UART4_RX   PC11
 *   -------  ---------  -----
 */

#define GPIO_UART4_TX (GPIO_UART4_TX_2|GPIO_SPEED_100MHz)
#define GPIO_UART4_RX (GPIO_UART4_RX_2|GPIO_SPEED_100MHz)

/* UART8:
 *
 * UART8 is connected to the "F9P GPS1" lines.
 *
 *   --------  ---------  -----
 *              STM32F7
 *   SIGNAME   FUNCTION   GPIO
 *   --------  ---------  -----
 *   GPS1_TX   UART8_TX   PE1
 *   GPS1_RX   UART8_RX   PE0
 *   --------  ---------  -----
 */

#define GPIO_UART8_TX (GPIO_UART8_TX_0|GPIO_SPEED_100MHz)
#define GPIO_UART8_RX (GPIO_UART8_RX_0|GPIO_SPEED_100MHz)

/* SPI
 *
 *  SPI3 is connected to the "W25Q128JV"
 * 
 *  PB4   SPI3_MISO
 *  PB5   SPI3_MOSI
 *  PB3   SPI3_SCK
 */

#define GPIO_SPI3_MISO   (GPIO_SPI3_MISO_1|GPIO_SPEED_50MHz)
#define GPIO_SPI3_MOSI   (GPIO_SPI3_MOSI_2|GPIO_SPEED_50MHz)
#define GPIO_SPI3_SCK    (GPIO_SPI3_SCK_1|GPIO_SPEED_50MHz)

/* I2C
 *
 *  I2C1 is connected to the "External Bus"
 * 
 *  PB8   I2C1_SCL
 *  PB9   I2C1_SDA
 * 
 *
 *  I2C3 is connected to the "Common Internal Bus"
 * 
 *  PA8   I2C3_SCL
 *  PC9   I2C3_SDA
 */

#define GPIO_I2C1_SCL (GPIO_I2C1_SCL_2|GPIO_SPEED_50MHz)
#define GPIO_I2C1_SDA (GPIO_I2C1_SDA_2|GPIO_SPEED_50MHz)

#define GPIO_I2C3_SCL (GPIO_I2C3_SCL_1|GPIO_SPEED_50MHz)
#define GPIO_I2C3_SDA (GPIO_I2C3_SDA_1|GPIO_SPEED_50MHz)

/* The STM32F7 connects to a Microchip KSZ8081RNA PHY using these pins:
 *
 *   STM32F7  BOARD        KSZ8081
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PB11     RMII_TX_EN   TXEN
 *   PB12     RMII_TXD0    TXD0
 *   PB13     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 * 
 * The PHY address is 0.
 * 
 */

#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_1|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_1|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_1|GPIO_SPEED_100MHz)

/* CAN Bus  */

#define GPIO_CAN1_TX          (GPIO_CAN1_TX_3|GPIO_SPEED_50MHz)     /* PD1  */
#define GPIO_CAN1_RX          (GPIO_CAN1_RX_3|GPIO_SPEED_50MHz)     /* PD0  */

/* SDMMC1 */

#define GPIO_SDMMC1_CK        (GPIO_SDMMC1_CK_0|GPIO_SPEED_50MHz)   /* PC12 */
#define GPIO_SDMMC1_CMD       (GPIO_SDMMC1_CMD_0|GPIO_SPEED_50MHz)  /* PD2  */
#define GPIO_SDMMC1_D0        (GPIO_SDMMC1_D0_0|GPIO_SPEED_50MHz)   /* PC8  */

/* OTGFS */

#define GPIO_OTGFS_DM         (GPIO_OTGFS_DM_0|GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP         (GPIO_OTGFS_DP_0|GPIO_SPEED_100MHz)

#endif  /* __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_INCLUDE_BOARD_H */
