#include "ch.h"
#include "hal.h"

#include "telemetry.h"
#include "usbcfg.h"
#include "mavlink.h"
#include "chprintf.h"
#include "bme280.h"

#define SERIAL_DEVICE SDU1

mavlink_system_t mavlink_system;
uint8_t bufS[MAVLINK_MAX_PACKET_LEN]; //Send buffer

static virtual_timer_t led_vt; //Timer for rx

void handle_1hz(void);
void handle_mavlink_message(mavlink_message_t msg);
void blink(void);

static void led_cb(void *arg) {
    (void) arg;
	palClearPad(GPIOB, GPIOB_STATUS_LED);
}

void blink() {
	palSetPad(GPIOB, GPIOB_STATUS_LED);
	chVTSet(&led_vt, MS2ST(50), led_cb, NULL);
}

/*
 * Mavlink receive
 */
static THD_WORKING_AREA(waMavlinkThread, 2000);
static THD_FUNCTION(MavlinkThread, arg) {

	(void) arg;
	static mavlink_message_t msg;
	static mavlink_status_t status;

	event_listener_t serialData;
	eventflags_t flags;
	chEvtRegisterMask((event_source_t *) chnGetEventSource(&SERIAL_DEVICE),
			&serialData, EVENT_MASK(1));

	chRegSetThreadName("mavlink");
	while (true) {
		chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(10));
		flags = chEvtGetAndClearFlags(&serialData);
		if(flags & CHN_INPUT_AVAILABLE)
		{
			msg_t charData;
			do {
				charData = chnGetTimeout(&SERIAL_DEVICE, TIME_IMMEDIATE);
				blink(); //Blink led for received data
				if (mavlink_parse_char(MAVLINK_COMM_0, charData, &msg, &status)) {
					handle_mavlink_message(msg);
				}
			} while(charData != Q_TIMEOUT);
		}
	}
	/* This point may be reached if shut down is requested. */
}

static THD_WORKING_AREA(waMavlinkTx, 1000);
static THD_FUNCTION(MavlinkTx, arg) {
    (void) arg;

	mavlink_message_t msgs;
	uint16_t len;
	while(true) {
        float temp = get_tempeture();
        float baro = get_baro();
        float hum = get_humidity();

        mavlink_msg_gas_sensor_board_pack(mavlink_system.sysid,
                mavlink_system.compid, &msgs, hum, temp, baro, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f);
        len = mavlink_msg_to_send_buffer(bufS, &msgs);
        chnWriteTimeout(&SERIAL_DEVICE, bufS, len, MS2ST(100));

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
        chnWriteTimeout(&SERIAL_DEVICE, bufS, len, MS2ST(100));

        chThdSleepMilliseconds(1000);
	}
}

void handle_mavlink_message(mavlink_message_t msg) {
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
			mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msgs,
				"TEST1\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0);
			len = mavlink_msg_to_send_buffer(bufS, &msgs);
			chnWriteTimeout(&SERIAL_DEVICE, bufS, len, MS2ST(100));
			mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msgs,
				"TEST2\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 1);
			len = mavlink_msg_to_send_buffer(bufS, &msgs);
			chnWriteTimeout(&SERIAL_DEVICE, bufS, len, MS2ST(100));
			mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msgs,
				"TEST3\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 2);
			len = mavlink_msg_to_send_buffer(bufS, &msgs);
			chnWriteTimeout(&SERIAL_DEVICE, bufS, len, MS2ST(100));
		}
		case MAVLINK_MSG_ID_DATA_STREAM: {
		    mavlink_data_stream_t decode;
		    mavlink_msg_data_stream_decode(&msg, &decode);
		}
	}
}

void init_telemetry() {
	mavlink_system.sysid = 3;
	mavlink_system.compid = MAV_COMP_ID_CAMERA;

	palClearPad(GPIOB, GPIOB_STATUS_LED);
	chVTObjectInit(&led_vt);

	chThdCreateStatic(waMavlinkThread, sizeof(waMavlinkThread), NORMALPRIO + 1,
			MavlinkThread, NULL);
	chThdCreateStatic(waMavlinkTx, sizeof(waMavlinkTx), NORMALPRIO + 1,
	            MavlinkTx, NULL);
}

