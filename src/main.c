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

#include "ch.h"
#include "hal.h"

#include "bme280.h"
#include "usbcfg.h"
#include "chprintf.h"
#include "telemetry.h"
#include "analog.h"
//#include "logging.h"


static const I2CConfig i2cconfig = { OPMODE_I2C, 100000, //100kHz
        STD_DUTY_CYCLE };

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    //Initialize interfaces
    sdStart(&SD1, NULL);

    i2cStart(&I2CD1, &i2cconfig);
    init_bme280();

    //Initialize USB serial
    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);

    /*chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
            NULL);
*/
    init_analog();
    measure_sensors();
    //init_logging();
    //write_to_file();
    init_telemetry(); //Init last to have all mtx objects init

    /*
     * Normal main() thread activity, in this demo it does nothing except
     * sleeping in a loop and check the button state, when the button is
     * pressed the test procedure is launched.
     */
    while (true) {
        measure_sensors();
        bme280_measurement();
        chThdSleepMilliseconds(1000);
    }
}
