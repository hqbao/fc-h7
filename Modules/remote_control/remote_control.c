#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

static int g_rc_values[16];

static void on_external_message(uint8_t *data, size_t size) {
	memcpy(g_rc_values, data, size);
}

static void handle_external_message(uint8_t *data, size_t size) {
	publish(COMMAND_SET_TARGET_ORIENTATION, (uint8_t*)g_rc_values, 8 * sizeof(int));
}

void remote_control_setup(void) {
	subscribe(EXTERNAL_MESSAGE, on_external_message);
	subscribe(SCHEDULER_100HZ, handle_external_message);
}
