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

static void led_cb(void *arg) {
	palClearPad(GPIOB, GPIOB_STATUS_LED);
}

void blink() {
	palSetPad(GPIOB, GPIOB_STATUS_LED);
	chVTSet(&led_vt, MS2ST(50), led_cb, NULL);
}

/*
 * Mavlink receive and hearth beat thread
 */
static THD_WORKING_AREA(waMavlinkThread, 5000);
static THD_FUNCTION(MavlinkThread, arg) {

	(void) arg;
	static mavlink_message_t msg;
	static mavlink_status_t status;
	uint8_t bufS[MAVLINK_MAX_PACKET_LEN];

	systime_t last_hearth_beat = chVTGetSystemTime();

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
		systime_t now = chVTGetSystemTime();
		systime_t next_t = last_hearth_beat + MS2ST(1000);
		if (last_hearth_beat + MS2ST(1000) < now) {
			handle_1hz();
			last_hearth_beat = now;
		}
	}
	/* This point may be reached if shut down is requested. */
}

void handle_1hz(void) {
	mavlink_message_t msgs;
	uint16_t len;
	//chMtxLock(&bme280_data);
	float temp = get_tempeture();
	float baro = get_baro();
	float hum = get_humidity();
	mavlink_msg_nav_controller_output_pack(mavlink_system.sysid,
			mavlink_system.compid, &msgs, ST2MS(chVTGetSystemTime()), temp,
			baro, 0, 0, 0, hum, 0.0f);
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
	}
}

void init_telemetry() {
	mavlink_system.sysid = 3;
	mavlink_system.compid = MAV_COMP_ID_CAMERA;

	palClearPad(GPIOB, GPIOB_STATUS_LED);
	chVTObjectInit(&led_vt);

	chThdCreateStatic(waMavlinkThread, sizeof(waMavlinkThread), NORMALPRIO + 1,
			MavlinkThread, NULL);
}

