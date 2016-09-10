#include "ch.h"
#include "hal.h"

#include "telemetry.h"
#include "usbcfg.h"
#include "mavlink_bridge.h" /* Has to be before mavlink.h */
#include "mavlink.h"
#include "chprintf.h"
#include "bme280.h"
#include "analog.h"

#define SERIAL_DEVICE SDU1

static virtual_timer_t led_vt; //Timer for rx
position_t last_pos;

void requset_gps_data_stream(void);
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


	while(true) {
        float temp = get_tempeture();
        float baro = get_baro();
        float hum = get_humidity();
        float co_gas, no2_gas, nh3_gas, no_gas, so2_gas;

        get_analog_sensor_values(&co_gas, &no2_gas, &nh3_gas, &no_gas, &so2_gas);

        mavlink_msg_gas_sensor_board_send(MAVLINK_COMM_0, hum, temp, baro, nh3_gas, no2_gas, co_gas, no_gas, so2_gas);

        // Define the system type, in this case an airplane
        uint8_t system_type = MAV_TYPE_GENERIC;
        uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

        uint8_t system_mode = MAV_MODE_AUTO_ARMED; ///< Booting up
        uint32_t custom_mode = 0;   ///< Custom mode, can be defined by user/adopter
        uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight

        // Pack the message
        mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type, system_mode, custom_mode, system_state);
        requset_gps_data_stream();

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
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST1\0", 1.0f, MAV_PARAM_TYPE_UINT8, 3, 0);
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST2\0", 1.0f, MAV_PARAM_TYPE_UINT8, 3, 1);
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST3\0", 1.0f, MAV_PARAM_TYPE_UINT8, 3, 2);
			break;
		}
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
		    mavlink_global_position_int_t decode;
		    mavlink_msg_global_position_int_decode(&msg,  &decode);
		    last_pos.lat = decode.lat;
		    last_pos.lon = decode.lon;
		    last_pos.alt = decode.alt;
		    break;
		}
		case MAVLINK_MSG_ID_DATA_STREAM: {
		    mavlink_data_stream_t decode;
		    mavlink_msg_data_stream_decode(&msg, &decode);
		    break;
		}
	}
}

void requset_gps_data_stream(void){
    /* Request stream with GPS coordinates */
    /* We should detect autopilot system  and use its sysid and comid and then requset stream
     * also use timeouts for rerequest etc. */
    mavlink_msg_request_data_stream_send(MAVLINK_COMM_0, 1, 1, MAV_DATA_STREAM_POSITION, 5, 1);
}

void init_telemetry() {
/*	mavlink_system.sysid = 3;
	mavlink_system.compid = MAV_COMP_ID_CAMERA; */

	palClearPad(GPIOB, GPIOB_STATUS_LED);
	chVTObjectInit(&led_vt);

	chThdCreateStatic(waMavlinkThread, sizeof(waMavlinkThread), NORMALPRIO + 1,
			MavlinkThread, NULL);
	chThdCreateStatic(waMavlinkTx, sizeof(waMavlinkTx), NORMALPRIO + 1,
	            MavlinkTx, NULL);
}

