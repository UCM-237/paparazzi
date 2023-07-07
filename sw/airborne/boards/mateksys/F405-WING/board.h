/*
    ChibiOS/RT - Copyright (C) Alfredo Gonzalez Calvin alfredgo@ucm.es

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/* Compiles and all pins are specified. However it does not work */

/*
 * Setup for matekf405-wing board 
 * Note the MCU used is xxxxx
 */

/*
 * Board identifier.
 */
#define BOARD_MATEK_F405_WING
#define BOARD_NAME  "Matek f405 wing"

/*
 * Board oscillators-related settings.
 * NOTE: LSE NOT? fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0 // originally was 32000 ?
#endif


#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000
#endif

/*
#define STM32_LSEDRV                (3U << 3U)
*/
/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header file stm32f4xx.h. (mcuconf.h)
 */
#define STM32F405xx

/* This is for STM32F045xx */


 
/*
 * IO lines assignments. Trying to do it by module (easier for me...)
 */

/**** ALL PINS FROM BARE METAL CONFIGURED. THIS IS DONE *****/

// UART DEFINITIONS 
#define LINE_UART1_RX				PAL_LINE(GPIOA, 10U)
#define LINE_UART1_TX				PAL_LINE(GPIOA, 9U)

#define LINE_UART2_RX				PAL_LINE(GPIOA, 3U)
#define LINE_UART2_TX				PAL_LINE(GPIOA, 2U)

#define LINE_UART3_RX				PAL_LINE(GPIOC, 11U)
#define LINE_UART3_TX				PAL_LINE(GPIOC, 10U)

#define LINE_UART4_RX				PAL_LINE(GPIOA, 1U)
#define LINE_UART4_TX				PAL_LINE(GPIOA, 0U)

#define LINE_UART5_RX				PAL_LINE(GPIOC, 2U)
#define LINE_UART5_TX				PAL_LINE(GPIOD, 12U)

#define LINE_UART6_RX				PAL_LINE(GPIOC, 7U)
#define LINE_UART6_TX				PAL_LINE(GPIOC, 6U)


// SPI DEFINITIONS 
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U) 
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U) 
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOA, 7U) 
#define 	LINE_SPI1_CS			PAL_LINE(GPIOA, 4U)

#define	LINE_SPI2_SCK                  PAL_LINE(GPIOB, 13U) 
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOC, 2U) 
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOC, 3U) 
#define 	LINE_SPI2_CS			PAL_LINE(GPIOB, 12U)

#define	LINE_SPI3_SCK                  PAL_LINE(GPIOB, 3U)
#define	LINE_SPI3_MISO                 PAL_LINE(GPIOB, 4U)
#define	LINE_SPI3_MOSI                 PAL_LINE(GPIOB, 5U)
#define 	LINE_SPI3_CS			PAL_LINE(GPIOC, 14U)

// I2C definitons 
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U) 
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 9U) 

#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U) 
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U) 

// Servo definitions 
#define	LINE_S1                        PAL_LINE(GPIOB, 7U) 
#define	LINE_S2                        PAL_LINE(GPIOB, 6U) 
#define	LINE_S3                        PAL_LINE(GPIOB, 0U) 
#define	LINE_S4                        PAL_LINE(GPIOB, 1U) 
#define	LINE_S5                        PAL_LINE(GPIOC, 8U) 
#define	LINE_S6                        PAL_LINE(GPIOC, 9U) 
#define	LINE_S7                        PAL_LINE(GPIOB, 14U) 
#define	LINE_S8                        PAL_LINE(GPIOB, 15U) 
#define	LINE_S9                        PAL_LINE(GPIOA, 8U) 

// LEDS
#define	LINE_LED1                      PAL_LINE(GPIOA, 14U) 
#define	LINE_LED2                      PAL_LINE(GPIOD, 13U) 

// Current and voltaje measurements
#define	LINE_CURRENT_MEAS              PAL_LINE(GPIOC, 1U) // ADC 2
#define	LINE_VBAT_MEAS                 PAL_LINE(GPIOC, 0U) 

// oscillator
#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

// TODO: Documentation for every port

/* PORT A SETUP: */
/*
 GPIA0: UART4_RX
 GPIOA0 : UART4_TX (ALT)
 GPIOA1 : UART4_RX (ALT)
 GPIOA2 : UART2_TX (ALT)
 GPIOA3 : UART2_RX (ALT)
 GPIOA4 : SPI1 CS  (OUT)
 GPIOA5 : SPI1 SCK (ALT)
 GPIOA6 : SPI1 MISO (ALT)
 GPIOA7 : SPI1 MOSI (ALT)
 GPIOA8 : SERVO 9   (ALT)
 GPIOA9 : UART1_TX  (ALT)
 GPIOA10: UART1_RX  (ALT)
 GPIOA11: --
 GPIOA12: --
 GPIOA13: --
 GPIOA14: LED1	     (OUT)
 GPIOA15: --
*/
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(0)| \
                                     PIN_MODE_ALTERNATE(1) | \
                                     PIN_MODE_ALTERNATE(2) | \
                                     PIN_MODE_ALTERNATE(3)     | \
                                     PIN_MODE_OUTPUT(4) | \
                                     PIN_MODE_ALTERNATE(5) | \
                                     PIN_MODE_ALTERNATE(6)     | \
                                     PIN_MODE_ALTERNATE(7) | \
                                     PIN_MODE_ALTERNATE(8)    | \
                                     PIN_MODE_ALTERNATE(9)     | \
                                     PIN_MODE_ALTERNATE(10)    | \
                                     PIN_MODE_OUTPUT(11)    | \
                                     PIN_MODE_OUTPUT(12)    | \
                                     PIN_MODE_OUTPUT(13)    | \
                                     PIN_MODE_OUTPUT(14)    | \
                                     PIN_MODE_OUTPUT(15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(0) | \
                                     PIN_OTYPE_PUSHPULL(1)  | \
                                     PIN_OTYPE_PUSHPULL(2)  | \
                                     PIN_OTYPE_PUSHPULL(3)  | \
                                     PIN_OTYPE_PUSHPULL(4)  | \
                                     PIN_OTYPE_PUSHPULL(5)  | \
                                     PIN_OTYPE_PUSHPULL(6)  | \
                                     PIN_OTYPE_PUSHPULL(7)  | \
                                     PIN_OTYPE_PUSHPULL(8) | \
                                     PIN_OTYPE_PUSHPULL(9)  | \
                                     PIN_OTYPE_PUSHPULL(10) | \
                                     PIN_OTYPE_PUSHPULL(11) | \
                                     PIN_OTYPE_PUSHPULL(12) | \
                                     PIN_OTYPE_PUSHPULL(13) | \
                                     PIN_OTYPE_PUSHPULL(14) | \
                                     PIN_OTYPE_PUSHPULL(15))
                              
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_SPEED_HIGH(0) | \
                                     PIN_OSPEED_SPEED_HIGH(1)  | \
                                     PIN_OSPEED_SPEED_HIGH(2)  | \
                                     PIN_OSPEED_SPEED_HIGH(3)  | \
                                     PIN_OSPEED_SPEED_HIGH(4)  | \
                                     PIN_OSPEED_SPEED_HIGH(5)  | \
                                     PIN_OSPEED_SPEED_HIGH(6)  | \
                                     PIN_OSPEED_SPEED_HIGH(7)  | \
                                     PIN_OSPEED_SPEED_HIGH(8)  | \
                                     PIN_OSPEED_SPEED_HIGH(9)  | \
                                     PIN_OSPEED_SPEED_HIGH(10) | \
                                     PIN_OSPEED_SPEED_VERYLOW(11) | \
                                     PIN_OSPEED_SPEED_VERYLOW(12) | \
                                     PIN_OSPEED_SPEED_VERYLOW(13) | \
                                     PIN_OSPEED_SPEED_VERYLOW(14) | \
                                     PIN_OSPEED_SPEED_VERYLOW(15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(0) | \
                                     PIN_PUPDR_FLOATING(1)    | \
                                     PIN_PUPDR_FLOATING(2)    | \
                                     PIN_PUPDR_FLOATING(3)    | \
                                     PIN_PUPDR_FLOATING(4)  | \
                                     PIN_PUPDR_FLOATING(5)  | \
                                     PIN_PUPDR_FLOATING(6)  | \
                                     PIN_PUPDR_FLOATING(7)  | \
                                     PIN_PUPDR_FLOATING(8)  | \
                                     PIN_PUPDR_FLOATING(9)  | \
                                     PIN_PUPDR_FLOATING(10) | \
                                     PIN_PUPDR_PULLDOWN(11) | \
                                     PIN_PUPDR_PULLDOWN(12) | \
                                     PIN_PUPDR_PULLDOWN(13) | \
                                     PIN_PUPDR_FLOATING(14) | \
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(0) | \
                                     PIN_ODR_HIGH(1)  | \
                                     PIN_ODR_HIGH(2)  | \
                                     PIN_ODR_HIGH(3)  | \
                                     PIN_ODR_HIGH(4)  | \
                                     PIN_ODR_HIGH(5)  | \
                                     PIN_ODR_HIGH(6)  | \
                                     PIN_ODR_HIGH(7)  | \
                                     PIN_ODR_LOW(8)  | \
                                     PIN_ODR_HIGH(9)  | \
                                     PIN_ODR_HIGH(10) | \
                                     PIN_ODR_LOW(11) | \
                                     PIN_ODR_LOW(12) | \
                                     PIN_ODR_LOW(13) | \
                                     PIN_ODR_LOW(14) | \
                                     PIN_ODR_LOW(15))
              /* PORT A SETUP: */
/*
 GPIA0: UART4_RX
 GPIOA0 : UART4_TX (ALT)
 GPIOA1 : UART4_RX (ALT)
 GPIOA2 : UART2_TX (ALT)
 GPIOA3 : UART2_RX (ALT)
 GPIOA4 : SPI1 CS  (OUT)
 GPIOA5 : SPI1 SCK (ALT)
 GPIOA6 : SPI1 MISO (ALT)
 GPIOA7 : SPI1 MOSI (ALT)
 GPIOA8 : SERVO 9   (ALT)
 GPIOA9 : UART1_TX  (ALT)
 GPIOA10: UART1_RX  (ALT)
 GPIOA11: --
 GPIOA12: --
 GPIOA13: --
 GPIOA14: LED1	     (OUT)
 GPIOA15: --
*/                       
// Here the alternating functions from the TODO above                                     
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0, 8) | \
                                     PIN_AFIO_AF(1, 8)  | \
                                     PIN_AFIO_AF(2, 7)  | \
                                     PIN_AFIO_AF(3, 7)  | \
                                     PIN_AFIO_AF(4, 5)  | \
                                     PIN_AFIO_AF(5, 5)  | \
                                     PIN_AFIO_AF(6, 5)  | \
                                     PIN_AFIO_AF(7, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8, 1)  | \
                                     PIN_AFIO_AF(9, 7)   | \
                                     PIN_AFIO_AF(10, 7) | \
                                     PIN_AFIO_AF(11, 0) | \
                                     PIN_AFIO_AF(12, 0) | \
                                     PIN_AFIO_AF(13, 1)  | \
                                     PIN_AFIO_AF(14, 1)  | \
                                     PIN_AFIO_AF(15, 2))
                                     
                                     
                                     
                                    
/* PORT B SETUP: */
/* 
 GPIOB0 : SERVO 3 (ALT)
 GPIOB1 : SERVO 4 (ALT)
 GPIOB2 :  --
 GPIOB3 : SPI3 SCK (ALT)
 GPIOB4 : SPI3 MISO (ALT) 
 GPIOB5 : SPI3 MOSI (ALT)
 GPIOB6 : SERVO 2 (ALT)
 GPIOB7 : SERVO 1 (ALT)
 GPIOB8 : IC1 SCL (ALT)
 GPIOB9 : IC1 SDA (ALT)
 GPIOB10: IC2 SCL (ALT)
 GPIOB11: IC2 SDA (ALT) 
 GPIOB12: SPI2 CS (OUT)
 GPIOB13: SPI2 SCK (ALT)
 GPIOB14: SERVO 7  (ALT)
 GPIOB15: SERVO 8  (ALT)
*/


#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(0) |           \
                                     PIN_MODE_ALTERNATE(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_ALTERNATE(3) |        \
                                     PIN_MODE_ALTERNATE(4) |           \
                                     PIN_MODE_ALTERNATE(5) |           \
                                     PIN_MODE_ALTERNATE(6) |        \
                                     PIN_MODE_ALTERNATE(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |        \
                                     PIN_MODE_ALTERNATE(10) |         \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_OUTPUT(12) |          \
                                     PIN_MODE_ALTERNATE(13) |          \
                                     PIN_MODE_ALTERNATE(14) |          \
                                     PIN_MODE_ALTERNATE(15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |        \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |     \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_SPEED_HIGH(0) |          \
                                     PIN_OSPEED_SPEED_HIGH(1) |          \
                                     PIN_OSPEED_SPEED_VERYLOW(2) |          \
                                     PIN_OSPEED_SPEED_HIGH(3) |           \
                                     PIN_OSPEED_SPEED_HIGH(4) |          \
                                     PIN_OSPEED_SPEED_HIGH(5) |          \
                                     PIN_OSPEED_SPEED_HIGH(6) |           \
                                     PIN_OSPEED_SPEED_HIGH(7) |          \
                                     PIN_OSPEED_SPEED_HIGH(8) |          \
                                     PIN_OSPEED_SPEED_HIGH(9) |           \
                                     PIN_OSPEED_SPEED_HIGH(10) |        \
                                     PIN_OSPEED_SPEED_HIGH(11) |         \
                                     PIN_OSPEED_SPEED_HIGH(12) |         \
                                     PIN_OSPEED_SPEED_HIGH(13) |         \
                                     PIN_OSPEED_SPEED_HIGH(14) |         \
                                     PIN_OSPEED_SPEED_HIGH(15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(0) |         \
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_PULLDOWN(2) |         \
                                     PIN_PUPDR_FLOATING(3) |        \
                                     PIN_PUPDR_FLOATING(4) |         \
                                     PIN_PUPDR_FLOATING(5) |         \
                                     PIN_PUPDR_FLOATING(6) |        \
                                     PIN_PUPDR_FLOATING(7) |         \
                                     PIN_PUPDR_FLOATING(8) |         \
                                     PIN_PUPDR_FLOATING(9) |        \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |        \
                                     PIN_PUPDR_FLOATING(13) |        \
                                     PIN_PUPDR_FLOATING(14) |        \
                                     PIN_PUPDR_FLOATING(15))
                                     
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(0) |             \
                                     PIN_ODR_LOW(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |              \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_LOW(6) |              \
                                     PIN_ODR_LOW(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |              \
                                     PIN_ODR_HIGH(10) |           \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_LOW(14) |            \
                                     PIN_ODR_LOW(15))
                                     
// Here the alternating functions from the TODO above
/* PORT B SETUP: */
/* 
 GPIOB0 : SERVO 3 (ALT)
 GPIOB1 : SERVO 4 (ALT)
 GPIOB2 :  --
 GPIOB3 : SPI3 SCK (ALT)
 GPIOB4 : SPI3 MISO (ALT) 
 GPIOB5 : SPI3 MOSI (ALT)
 GPIOB6 : SERVO 2 (ALT)
 GPIOB7 : SERVO 1 (ALT)
 GPIOB8 : IC1 SCL (ALT)
 GPIOB9 : IC1 SDA (ALT)
 GPIOB10: IC2 SCL (ALT)
 GPIOB11: IC2 SDA (ALT) 
 GPIOB12: SPI2 CS (OUT)
 GPIOB13: SPI2 SCK (ALT)
 GPIOB14: SERVO 7  (ALT)
 GPIOB15: SERVO 8  (ALT)
*/

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(0, 2) |           \
                                     PIN_AFIO_AF(1, 2) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 6) |            \
                                     PIN_AFIO_AF(4, 6) |           \
                                     PIN_AFIO_AF(5, 6) |           \
                                     PIN_AFIO_AF(6, 2) |            \
                                     PIN_AFIO_AF(7, 2))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8, 4) |           \
                                     PIN_AFIO_AF(9, 4) |            \
                                     PIN_AFIO_AF(10, 4) |         \
                                     PIN_AFIO_AF(11, 4) |          \
                                     PIN_AFIO_AF(12, 5) |          \
                                     PIN_AFIO_AF(13, 5) |          \
                                     PIN_AFIO_AF(14, 9) |          \
                                     PIN_AFIO_AF(15, 9))

/* PORT C SETUP: */
/* 
 GPIOC0 : LINE_VBAT_MEAS (ANALOG)
 GPIOC1 : LINE_CURRENT_MEAS (ANALOG)
 GPIOC2 : SPI2 MISO (ALT)
 GPIOC3 : SPI2 MOSI (ALT)
 GPIOC4 : --
 GPIOC5 : --
 GPIOC6 : UART6 TX (ALT)
 GPIOC7 : UART6 RX (ALT)
 GPIOC8 : SERVO 5 (ATL)
 GPIOC9 : SERVO 6 (ALT)
 GPIOC10: UART3 TX (ALT)
 GPIOC11: UART3 RX (ALT)
 GPIOC12: UART5 TX (ALT)
 GPIOC13: UART5 RX (ALT)
 GPIOC14: SPI3 CS (OUTPUT)
 GPIOC15: --
*/
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(0) |\
                                     PIN_MODE_ANALOG(1) |           \
                                     PIN_MODE_ALTERNATE(2) |           \
                                     PIN_MODE_ALTERNATE(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_ALTERNATE(7) |       \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |           \
                                     PIN_MODE_ALTERNATE(10) |       \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_ALTERNATE(12) |       \
                                     PIN_MODE_ALTERNATE(13) |          \
                                     PIN_MODE_OUTPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |    \
                                     PIN_OTYPE_OPENDRAIN(4) |       \
                                     PIN_OTYPE_OPENDRAIN(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |       \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |       \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_OPENDRAIN(15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_SPEED_VERYLOW(0) |\
                                     PIN_OSPEED_SPEED_VERYLOW(1) |          \
                                     PIN_OSPEED_SPEED_HIGH(2) |          \
                                     PIN_OSPEED_SPEED_HIGH(3) |       \
                                     PIN_OSPEED_SPEED_VERYLOW(4) |          \
                                     PIN_OSPEED_SPEED_VERYLOW(5) |          \
                                     PIN_OSPEED_SPEED_HIGH(6) |          \
                                     PIN_OSPEED_SPEED_HIGH(7) |          \
                                     PIN_OSPEED_SPEED_HIGH(8) |          \
                                     PIN_OSPEED_SPEED_HIGH(9) |          \
                                     PIN_OSPEED_SPEED_HIGH(10) |          \
                                     PIN_OSPEED_SPEED_HIGH(11) |         \
                                     PIN_OSPEED_SPEED_HIGH(12) |          \
                                     PIN_OSPEED_SPEED_HIGH(13) |         \
                                     PIN_OSPEED_SPEED_HIGH(14) |         \
                                     PIN_OSPEED_SPEED_VERYLOW(15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0) |\
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |      \
                                     PIN_PUPDR_PULLUP(4) |         \
                                     PIN_PUPDR_PULLUP(5) |         \
                                     PIN_PUPDR_FLOATING(6) |         \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |         \
                                     PIN_PUPDR_FLOATING(9) |         \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |       \
                                     PIN_PUPDR_FLOATING(13) |        \
                                     PIN_PUPDR_FLOATING(14) |        \
                                     PIN_PUPDR_PULLUP(15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(0) |  \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |          \
                                     PIN_ODR_LOW(4) |             \
                                     PIN_ODR_LOW(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |             \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |             \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_LOW(15))
          /* PORT C SETUP: */
/* 
 GPIOC0 : LINE_VBAT_MEAS (ANALOG)
 GPIOC1 : LINE_CURRENT_MEAS (ANALOG)
 GPIOC2 : SPI2 MISO (ALT)
 GPIOC3 : SPI2 MOSI (ALT)
 GPIOC4 : --
 GPIOC5 : --
 GPIOC6 : UART6 TX (ALT)
 GPIOC7 : UART6 RX (ALT)
 GPIOC8 : SERVO 5 (ATL)
 GPIOC9 : SERVO 6 (ALT)
 GPIOC10: UART3 TX (ALT)
 GPIOC11: UART3 RX (ALT)
 GPIOC12: UART5 TX (ALT)
 GPIOC13: UART5 RX (ALT)
 GPIOC14: SPI3 CS (OUTPUT)
 GPIOC15: --
*/                                                      
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0, 0) |\
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 5) |           \
                                     PIN_AFIO_AF(3, 5) |        \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 8) |           \
                                     PIN_AFIO_AF(7, 8))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8, 3) |           \
                                     PIN_AFIO_AF(9, 3) |           \
                                     PIN_AFIO_AF(10, 7) |           \
                                     PIN_AFIO_AF(11, 7) |          \
                                     PIN_AFIO_AF(12, 8) |           \
                                     PIN_AFIO_AF(13, 8) |          \
                                     PIN_AFIO_AF(14, 5) |          \
                                     PIN_AFIO_AF(15, 0))


/* Don't care, not used by matekf405 */

/* 
 * GPIOD setup:
 *
*/
#define VAL_GPIOD_MODER             0x00000000
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0x00000000
#define VAL_GPIOD_PUPDR             0x55555555 // all pullup
#define VAL_GPIOD_ODR               0xFFFFFFFF
#define VAL_GPIOD_AFRL              0x00000000
#define VAL_GPIOD_AFRH              0x00000000

/* 
 * GPIOE setup:
 *
*/
#define VAL_GPIOE_MODER             0x00000000
#define VAL_GPIOE_OTYPER            0x00000000
#define VAL_GPIOE_OSPEEDR           0x00000000
#define VAL_GPIOE_PUPDR             0x55555555 // all pullup
#define VAL_GPIOE_ODR               0xFFFFFFFF
#define VAL_GPIOE_AFRL              0x00000000
#define VAL_GPIOE_AFRH              0x00000000
/*
 * GPIOF setup:
 *
*/
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0x00000000
#define VAL_GPIOF_PUPDR             0x55555555 // all pullup
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * GPIOG setup:
 *
*/
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0x00000000
#define VAL_GPIOG_PUPDR             0x55555555 // all pullup
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000
/*
 * GPIOH setup:
 *
*/
#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(0) | \
					 PIN_MODE_ALTERNATE(1) | \
					 PIN_MODE_INPUT(2) | \
					 PIN_MODE_INPUT(3) | \
					 PIN_MODE_INPUT(4) | \
					 PIN_MODE_INPUT(5) | \
					 PIN_MODE_INPUT(6) | \
					 PIN_MODE_INPUT(7) | \
					 PIN_MODE_INPUT(8) | \
					 PIN_MODE_INPUT(9) | \
					 PIN_MODE_INPUT(10) | \
					 PIN_MODE_INPUT(11) | \
					 PIN_MODE_INPUT(12) | \
					 PIN_MODE_INPUT(13) | \
					 PIN_MODE_INPUT(14) | \
					 PIN_MODE_INPUT(15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(0) | \
					 PIN_OTYPE_PUSHPULL(1) | \
					 PIN_OTYPE_PUSHPULL(2) | \
					 PIN_OTYPE_PUSHPULL(3) | \
					 PIN_OTYPE_PUSHPULL(4) | \
					 PIN_OTYPE_PUSHPULL(5) | \
					 PIN_OTYPE_PUSHPULL(6) | \
					 PIN_OTYPE_PUSHPULL(7) | \
					 PIN_OTYPE_PUSHPULL(8) | \
					 PIN_OTYPE_PUSHPULL(9) | \
					 PIN_OTYPE_PUSHPULL(10) | \
					 PIN_OTYPE_PUSHPULL(11) | \
					 PIN_OTYPE_PUSHPULL(12) | \
					 PIN_OTYPE_PUSHPULL(13) | \
					 PIN_OTYPE_PUSHPULL(14) | \
					 PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(0) | \
					 PIN_OSPEED_SPEED_HIGH(1) | \
					 PIN_OSPEED_SPEED_VERYLOW(2) | \
					 PIN_OSPEED_SPEED_VERYLOW(3) | \
					 PIN_OSPEED_SPEED_VERYLOW(4) | \
					 PIN_OSPEED_SPEED_VERYLOW(5) | \
					 PIN_OSPEED_SPEED_VERYLOW(6) | \
					 PIN_OSPEED_SPEED_VERYLOW(7) | \
					 PIN_OSPEED_SPEED_VERYLOW(8) | \
					 PIN_OSPEED_SPEED_VERYLOW(9) | \
					 PIN_OSPEED_SPEED_VERYLOW(10) | \
					 PIN_OSPEED_SPEED_VERYLOW(11) | \
					 PIN_OSPEED_SPEED_VERYLOW(12) | \
					 PIN_OSPEED_SPEED_VERYLOW(13) | \
					 PIN_OSPEED_SPEED_VERYLOW(14) | \
					 PIN_OSPEED_SPEED_VERYLOW(15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(0) | \
					 PIN_PUPDR_FLOATING(1) | \
					 PIN_PUPDR_PULLDOWN(2) | \
					 PIN_PUPDR_PULLDOWN(3) | \
					 PIN_PUPDR_PULLDOWN(4) | \
					 PIN_PUPDR_PULLDOWN(5) | \
					 PIN_PUPDR_PULLDOWN(6) | \
					 PIN_PUPDR_PULLDOWN(7) | \
					 PIN_PUPDR_PULLDOWN(8) | \
					 PIN_PUPDR_PULLDOWN(9) | \
					 PIN_PUPDR_PULLDOWN(10) | \
					 PIN_PUPDR_PULLDOWN(11) | \
					 PIN_PUPDR_PULLDOWN(12) | \
					 PIN_PUPDR_PULLDOWN(13) | \
					 PIN_PUPDR_PULLDOWN(14) | \
					 PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOH_ODR                   (PIN_ODR_HIGH(0) | \
					 PIN_ODR_HIGH(1) | \
					 PIN_ODR_LOW(2) | \
					 PIN_ODR_LOW(3) | \
					 PIN_ODR_LOW(4) | \
					 PIN_ODR_LOW(5) | \
					 PIN_ODR_LOW(6) | \
					 PIN_ODR_LOW(7) | \
					 PIN_ODR_LOW(8) | \
					 PIN_ODR_LOW(9) | \
					 PIN_ODR_LOW(10) | \
					 PIN_ODR_LOW(11) | \
					 PIN_ODR_LOW(12) | \
					 PIN_ODR_LOW(13) | \
					 PIN_ODR_LOW(14) | \
					 PIN_ODR_LOW(15))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(0, 0) | \
					 PIN_AFIO_AF(1, 0) | \
					 PIN_AFIO_AF(2, 0) | \
					 PIN_AFIO_AF(3, 0) | \
					 PIN_AFIO_AF(4, 0) | \
					 PIN_AFIO_AF(5, 0) | \
					 PIN_AFIO_AF(6, 0) | \
					 PIN_AFIO_AF(7, 0))
					 
#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(8, 0) | \
					 PIN_AFIO_AF(9, 0) | \
					 PIN_AFIO_AF(10, 0) | \
					 PIN_AFIO_AF(11, 0) | \
					 PIN_AFIO_AF(12, 0) | \
					 PIN_AFIO_AF(13, 0) | \
					 PIN_AFIO_AF(14, 0) | \
					 PIN_AFIO_AF(15, 0))

/*
 * GPIOI setup:
 *
*/
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0x00000000
#define VAL_GPIOI_PUPDR             0x55555555 // all pullup
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000



/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

// ---- //

/*
 * Onboard LEDs
 */
// Onboard LEDs
/* STAT blue, on PB5 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* WARN red, on PB4 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

// LED STRIP 2812
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOA
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

// BEEPER
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO15
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

// ---- //


/* no voltage divider on board, adjust VoltageOfAdc in airframe file */
#define DefaultVoltageOfAdc(adc) (0.008830925*adc)
#define DefaultMilliAmpereOfAdc(adc) (25*adc)





/**
 * I2C defines TODO: getting DMA failiure -> check timers and DMA mapping
 */
#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif
#if I2C1_CLOCK_SPEED == 400000
#define I2C1_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C1 clock speed
#endif
#define I2C1_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           I2C1_DUTY_CYCLE,   \
           }

#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif
#if I2C2_CLOCK_SPEED == 400000
#define I2C2_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C2 clock speed
#endif
#define I2C2_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           I2C2_DUTY_CYCLE,   \
           }

/**
 * SPI Config
 * SPI1 si for sensors
 * SPI2 is for FRAM
 * SPI4 is external
 */

// SPI1_SLAVE0 -> slave select pin for the  (gyro AND ACC)
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO3
// SPI1_SLAVE1 -> slave select pin for the  (i2C BARO MAG)

/*
#define AF_S1                            2U
#define AF_LINE_S1                       2U
#define AF_S2                            2U
#define AF_LINE_S2                       2U
#define AF_S3                            2U
#define AF_LINE_S3                       2U
#define AF_S4                            2U
#define AF_LINE_S4                       2U
#define AF_SPI1_SCK                      5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_SPI1_MISO                     5U
#define AF_LINE_SPI1_MISO                5U
#define AF_SPI1_MOSI                     5U
#define AF_LINE_SPI1_MOSI                5U
*/

#define AF_UART1_TX                      7U
#define AF_LINE_UART1_TX                 7U
#define AF_UART1_RX                      7U
#define AF_LINE_UART1_RX                 7U
/*
#define AF_S5                            3U
#define AF_LINE_S5                       3U
#define AF_S6                            3U
#define AF_LINE_S6                       3U
#define AF_SPI3_SCK                      5U
#define AF_LINE_SPI3_SCK                 5U
#define AF_SPI3_MISO                     5U
#define AF_LINE_SPI3_MISO                5U
#define AF_SPI3_MOSI                     5U
#define AF_LINE_SPI3_MOSI                5U

#define AF_I2C1_SCL                      0U
#define AF_LINE_I2C1_SCL                 0U
#define AF_I2C1_SDA                      0U
#define AF_LINE_I2C1_SDA                 0U

#define AF_UART5_RX                      8U
#define AF_LINE_UART5_RX                 8U

#define AF_I2C2_SCL                      0U
#define AF_LINE_I2C2_SCL                 0U
#define AF_I2C2_SDA                      0U
#define AF_LINE_I2C2_SDA                 0U

#define AF_SPI2_SCK                      5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_SPI2_MISO                     5U
#define AF_LINE_SPI2_MISO                5U
#define AF_SPI2_MOSI                     5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_UART4_RX                      8U
#define AF_LINE_UART4_RX                 8U
#define AF_UART4_TX                      8U
#define AF_LINE_UART4_TX                 8U

#define AF_UART2_TX                      7U
#define AF_LINE_UART2_TX                 7U
#define AF_UART2_RX                      7U
#define AF_LINE_UART2_RX                 7U
#define AF_UART3_TX                      7U
#define AF_LINE_UART3_TX                 7U
#define AF_UART3_RX                      7U
#define AF_LINE_UART3_RX                 7U

#define AF_S7                            9U
#define AF_LINE_S7                       9U
#define AF_S8                            9U
#define AF_LINE_S8                       9U
#define AF_S9                            1U
#define AF_LINE_S9                       1U

#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U

#define S1_TIM	 	 4
#define S1_TIM_FN	 CH
#define S1_TIM_CH	 2
#define S1_TIM_AF	 2

#define S2_TIM	 	 4
#define S2_TIM_FN	 CH
#define S2_TIM_CH	 1
#define S2_TIM_AF	 2

#define S3_TIM	 	 3
#define S3_TIM_FN	 CH
#define S3_TIM_CH	 3
#define S3_TIM_AF	 2

#define S4_TIM		 5
#define S4_TIM_FN	 CH
#define S4_TIM_CH	 4
#define S4_TIM_AF	 2

#define S5_TIM	 	 8
#define S5_TIM_FN	 CH
#define S5_TIM_CH	 3
#define S5_TIM_AF	 3

#define S6_TIM	 	 8
#define S6_TIM_FN	 CH
#define S6_TIM_CH	 4
#define S6_TIM_AF	 3

#define S7_TIM		 4
#define S7_TIM_FN	 CH
#define S7_TIM_CH	 1
#define S7_TIM_AF	 9

#define S8_TIM	 	 4
#define S8_TIM_FN	 CH
#define S8_TIM_CH	 2
#define S8_TIM_AF	 9

#define S9_TIM	 	 4
#define S9_TIM_FN	 CH
#define S9_TIM_CH	 1
#define S9_TIM_AF	 1
*/

/*
#define VBAT_MEAS_ADC	 	 1
#define VBAT_MEAS_ADC_FN	 IN
#define VBAT_MEAS_ADC_IN	 12
#define CURRENT_MEAS_ADC	 1
#define CURRENT_MEAS_ADC_FN	 IN
#define CURRENT_MEAS_ADC_IN	 13
*/


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
 /*
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()
*/
#define BOARD_GROUP_DECLFOREACH(line, group) \
  static const ioline_t group ## _ARRAY[] = {group}; \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)

#define BOARD_GROUP_FOREACH(line, group) \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)


#define BOARD_GROUP_DECLFOR(array, index, group)  \
  static const ioline_t group ## _ARRAY[] = {group};    \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define BOARD_GROUP_FOR(array, index, group)  \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */




#endif /* _BOARD_H_ */
