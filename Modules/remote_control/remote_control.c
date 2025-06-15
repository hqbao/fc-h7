#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define PULSE_WIDTH_TO_DEGREE (90.0 / 490)

static uint8_t g_rc_data_raw[18] = {0};
static float g_rc_move_in[4] = {0};
static uint8_t g_rc_state[2] = {0};

static void on_internal_message(uint8_t *data, size_t size) {
	if (data[0] == 0x01) { // Command
		if (data[1] == 0x02) { // Set target
			memcpy(g_rc_data_raw, &data[4], 18);
		}
	}
}

static void handle_internal_message(uint8_t *data, size_t size) {
	g_rc_move_in[0] = (float)(*(int*)&g_rc_data_raw[0]) * PULSE_WIDTH_TO_DEGREE;
	g_rc_move_in[1] = (float)(*(int*)&g_rc_data_raw[4]) * PULSE_WIDTH_TO_DEGREE;
	g_rc_move_in[2] = (float)(*(int*)&g_rc_data_raw[8]) * PULSE_WIDTH_TO_DEGREE;
	g_rc_move_in[3] = (float)(*(int*)&g_rc_data_raw[12]) * PULSE_WIDTH_TO_DEGREE;

	g_rc_state[0] = g_rc_data_raw[16];
	g_rc_state[1] = g_rc_data_raw[17];

	publish(COMMAND_SET_STATE, g_rc_state, 2);
	publish(COMMAND_SET_MOVE_IN, g_rc_move_in, 16);
}

void remote_control_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_internal_message);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
