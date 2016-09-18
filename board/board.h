/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

/*
 * Setup for the Gas sensor board.
 */

/*
 * Board identifier.
 */
#define BOARD_KMTI_GAS_SENSOR
#define BOARD_NAME              "Gas Sensors Rev. A"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            16000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments.
 */
#define GPIOA_CARD_DETECT       3
#define GPIOA_CARD_PWR          4

#define GPIOB_SD_CS             0
#define GPIOB_STATUS_LED        5
#define GPIOB_1M_MES_SEL       10
#define GPIOB_100K_MES_SEL     11
#define GPIOB_6K8_MES_SEL      12
#define GPIOB_SEN_SEL3         13
#define GPIOB_SEN_SEL2         14
#define GPIOB_SEN_SEL1         15


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Analog input                      (RESIST_VOLT).
 * PA1  - Analog input                      (NO_GAS_ANALOG).
 * PA2  - Analog input                      (SO2_GAS_ANALOG).
 * PA3  - Digital input                     (CARD_DETECT).
 * PA4  - Push Pull output 2MHz             (CARD_PWR).
 * PA5  - Alternate Push Pull output 50MHz  (SPI1_SCK).
 * PA6  - Digital Input                     (SPI1_MISO).
 * PA7  - Alternate Push Pull output 50MHz  (SPI1_MOSI).
 * PA9  - Alternate Push Pull output 50MHz  (USART1_TX).
 * PA10 - Digital input with pull-up        (USART1_RX).
 * PA11 - Digital input                     (USB DM).
 * PA12 - Digital input                     (USB DP).
 */
#define VAL_GPIOACRL            0xB4B24000      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888448B8      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFF7

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB0  - Push Pull output 50MHz              (SD_CS).
 * PB1  - Push Pull output 2MHz               (STATUS_LED).
 * PB6  - Alternate Open Drain output 50MHz.  (I2C1_SCL).
 * PB7  - Alternate Open Drain output 50MHz.  (I2C1_SDA).
 * PB8  - Digital input with pull-up          (CAN_RX).
 * PB9  - Alternate Push Pull output 50MHz.   (CAN_TX).
 * PB10 - Digital input                       (1M_MESL_SEL).
 * PB11 - Digital input                       (100K_MES_SEL).
 * PB12 - Digital input                       (6K8_MES_SEL).
 * PB13 - Push Pull output 2MHz               (SEN_SEL1).
 * PB14 - Push Pull output 2MHz               (SEN_SEL2).
 * PB15 - Push Pull output 2MHz               (SEN_SEL3).
 */
#define VAL_GPIOBCRL            0xFF888821      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x222444B8      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC14 - Digital input (OSC32)
 * PC15 - Digital input (OSC32)
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x44888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) (void)//palClearPad(GPIOC, GPIOC_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) (void)//palSetPad(GPIOC, GPIOC_USB_DISC)

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
