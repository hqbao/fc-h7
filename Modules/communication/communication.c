#include "communication.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include "main.h"

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
    if (data[0] == 0x01) { // Orientation control
        publish(COMMAND_SET_TARGET_ORIENTATION, &data[4], 12);
    }
}

void communication_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_message_received);
}
