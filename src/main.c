#include "ch.h"
#include "hal.h"

#include "bme280.h"
#include "usbcfg.h"
#include "chprintf.h"
#include "telemetry.h"
#include "analog.h"

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
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);

    init_analog();
    measure_sensors();
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
