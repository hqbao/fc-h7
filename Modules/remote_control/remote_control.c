#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define PULSE_WIDTH_TO_DEGREE (90.0 / 490)

static uint8_t g_rc_data_raw[18] = {0};
static uint8_t g_rc_value[18] = {0};

static void on_internal_message(uint8_t *data, size_t size) {
	if (data[0] == 0x01) { // Command
		if (data[1] == 0x02) { // Set target
			memcpy(g_rc_data_raw, &data[4], 18);
		}
	}
}

static void handle_internal_message(uint8_t *data, size_t size) {
	float roll 	= (float)(*(int*)&g_rc_data_raw[0]) * PULSE_WIDTH_TO_DEGREE;
	float pitch = (float)(*(int*)&g_rc_data_raw[4]) * PULSE_WIDTH_TO_DEGREE;
	float yaw 	= (float)(*(int*)&g_rc_data_raw[8]) * PULSE_WIDTH_TO_DEGREE;
	float alt 	= (float)(*(int*)&g_rc_data_raw[12]) * PULSE_WIDTH_TO_DEGREE;

	memcpy(&g_rc_value[0], (uint8_t*)&roll, 4);
	memcpy(&g_rc_value[4], (uint8_t*)&pitch, 4);
	memcpy(&g_rc_value[8], (uint8_t*)&yaw, 4);
	memcpy(&g_rc_value[12], (uint8_t*)&alt, 4);
	g_rc_value[16] = g_rc_data_raw[16];
	g_rc_value[17] = g_rc_data_raw[17];

	publish(COMMAND_SET_MOVE_IN, g_rc_value, 18);
}

void remote_control_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_internal_message);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
