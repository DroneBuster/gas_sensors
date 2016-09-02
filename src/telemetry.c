#include "ch.h"
#include "hal.h"

#include "telemetry.h"
#include "usbcfg.h"
#include "mavlink.h"
#include "chprintf.h"
#include "bme280.h"

#define SERIAL_DEVICE SDU1

mavlink_system_t mavlink_system;

void handle_1hz(void);

/*
 * Mavlink receive and hearth beat thread
 */
static THD_WORKING_AREA(waMavlinkThread, 5000);
static THD_FUNCTION(MavlinkThread, arg) {

    (void) arg;
    uint8_t mavBuff[64];
    static mavlink_message_t msg;
    static mavlink_status_t status;
    uint8_t bufS[MAVLINK_MAX_PACKET_LEN];

    systime_t next_hearth_beat = chVTGetSystemTime();


    chRegSetThreadName("mavlink");
    while (true) {
        uint8_t bytesRead = chnReadTimeout(&SERIAL_DEVICE, mavBuff, 64,
                MS2ST(5));
        uint8_t i;
        //chprintf((BaseSequentialStream *)&SDU1, "Test");
        for (i = 0; i < bytesRead; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, mavBuff[i], &msg, &status)) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t pack;
                    mavlink_msg_heartbeat_decode(&msg, &pack);
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    //mavlink_param_value_t pack;
                    mavlink_message_t msgs;
                    uint16_t len;
                    mavlink_msg_param_value_pack(mavlink_system.sysid,
                            mavlink_system.compid, &msgs, "TEST1\0", 1.0f,
                            MAV_PARAM_TYPE_UINT8, 1, 0);
                    len = mavlink_msg_to_send_buffer(bufS, &msgs);
                    chnWrite(&SERIAL_DEVICE, bufS, len);
                    mavlink_msg_param_value_pack(mavlink_system.sysid,
                            mavlink_system.compid, &msgs, "TEST2\0", 1.0f,
                            MAV_PARAM_TYPE_UINT8, 1, 1);
                    len = mavlink_msg_to_send_buffer(bufS, &msgs);
                    chnWrite(&SERIAL_DEVICE, bufS, len);
                    mavlink_msg_param_value_pack(mavlink_system.sysid,
                            mavlink_system.compid, &msgs, "TEST3\0", 1.0f,
                            MAV_PARAM_TYPE_UINT8, 1, 2);
                    len = mavlink_msg_to_send_buffer(bufS, &msgs);
                    chnWrite(&SERIAL_DEVICE, bufS, len);
                }
                }
            }
        }
        if (next_hearth_beat < chVTGetSystemTime()) {
            handle_1hz();
            next_hearth_beat += MS2ST(1000);
        }
        chThdSleepMicroseconds(100);
    }
    /* This point may be reached if shut down is requested. */
}

void handle_1hz(void) {
    mavlink_message_t msgs;
    uint16_t len;
    uint8_t bufS[MAVLINK_MAX_PACKET_LEN];
    //chMtxLock(&bme280_data);
    float temp = get_tempeture();
    float baro = get_baro();
    float hum = get_humidity();
    mavlink_msg_nav_controller_output_pack(mavlink_system.sysid, mavlink_system.compid, &msgs, ST2MS(chVTGetSystemTime()),
            temp, baro, 0, 0, 0, hum, 0.0f);
    //chMtxUnlock(&bme280_data);
    len = mavlink_msg_to_send_buffer(bufS, &msgs);
    chnWrite(&SERIAL_DEVICE, bufS, len);

    // Define the system type, in this case an airplane
    uint8_t system_type = MAV_TYPE_GIMBAL;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_AUTO_ARMED; ///< Booting up
    uint32_t custom_mode = 0;   ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight

    // Pack the message
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid,
            &msgs, system_type, autopilot_type, system_mode, custom_mode,
            system_state);
    // Copy the message to the send buffer
    len = mavlink_msg_to_send_buffer(bufS, &msgs);
    chnWrite(&SERIAL_DEVICE, bufS, len);


}

void init_telemetry() {
    mavlink_system.sysid = 3;
    mavlink_system.compid = MAV_COMP_ID_CAMERA;

    chThdCreateStatic(waMavlinkThread, sizeof(waMavlinkThread), NORMALPRIO + 1,
            MavlinkThread, NULL);
}

