#include "logger.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <macro.h>
#include <string.h>

static vector3d_t g_linear_veloc = {0, 0, 0};

static void linear_veloc_update(uint8_t *data, size_t size) {
	memcpy(&g_linear_veloc, data, size);
}

static void logger_loop_25hz(uint8_t *data, size_t size) {
	static uint8_t g_output_msg[128] = {'d', 'b', 0x00 /* Info */, 0x05 /* Linear velocity */};
	int buf_idx = 6;

	int x = (int)g_linear_veloc.x;
	int y = (int)g_linear_veloc.y;
	int z = (int)g_linear_veloc.z;
	memcpy(&g_output_msg[buf_idx], &x, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &y, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &z, 4); buf_idx += 4;

	uint16_t payload_size = buf_idx - 6;
	memcpy(&g_output_msg[4], &payload_size, 2); // 2-byte checksum
	memset(&g_output_msg[buf_idx], 0, 2); // 2-byte checksum, no use

	platform_uart_send(UART_PORT1, g_output_msg, buf_idx + 2);
}

void logger_setup(void) {
	subscribe(SENSOR_LINEAR_VELOC, linear_veloc_update);
	subscribe(SCHEDULER_25HZ, logger_loop_25hz);
}

