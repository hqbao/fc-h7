#include "communication.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include "main.h"

static void on_message_received(uint8_t *data, size_t size) {
	if (data[0] == 0x01) { // External sensors
		if (data[1] == 0x00) { // Optical flow
			platform_toggle_led(0);
			publish(EXTERNAL_SENSOR_OPTFLOW, &data[4], 12);
		}
	}
    else if (data[0] == 0x02) { // Orientation control
    	if (data[1] == 0x00) { // Set target
			publish(COMMAND_SET_TARGET_ORIENTATION, &data[4], 12);
		}
    }
}

void communication_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_message_received);
}
