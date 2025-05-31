#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

static uint8_t g_rc_data[18] = {0};

static void on_internal_message(uint8_t *data, size_t size) {
	if (data[0] == 0x01) { // Command
		if (data[1] == 0x02) { // Set target
			memcpy(g_rc_data, &data[4], 18);
		}
	}
}

static void handle_internal_message(uint8_t *data, size_t size) {
	publish(COMMAND_SET_MOVE_IN, g_rc_data, 18);
}

void remote_control_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_internal_message);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
