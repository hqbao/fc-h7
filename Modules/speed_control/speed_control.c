#include "speed_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <macro.h>

static int g_output_speed[4] = {0, 0, 0, 0};

static void sc_setup(uint8_t *data, size_t size) {
	platform_dshot_init(DSHOT_PORT1);
	platform_dshot_init(DSHOT_PORT2);
	platform_dshot_init(DSHOT_PORT3);
	platform_dshot_init(DSHOT_PORT4);
}

static void sc_update(uint8_t *data, size_t size) {
	memcpy(g_output_speed, data, size);
	platform_dshot_send(DSHOT_PORT1, g_output_speed[0]);
	platform_dshot_send(DSHOT_PORT2, g_output_speed[1]);
	platform_dshot_send(DSHOT_PORT3, g_output_speed[2]);
	platform_dshot_send(DSHOT_PORT4, g_output_speed[3]);
}

void speed_control_setup(void) {
	subscribe(SPEED_CONTROL_SETUP, sc_setup);
	subscribe(SPEED_CONTROL_UPDATE, sc_update);
}
