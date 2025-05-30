#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

static int g_rc_values[16];

static void on_external_message(uint8_t *data, size_t size) {
	memcpy(g_rc_values, data, size);
}

static void send_control_command(int roll, int pitch, int yaw) {
	static uint8_t g_output_msg[128] = {'d', 'b', 0x01 /* Command */, 0x02 /* Control orientation */};
	int buf_idx = 6;
	memcpy(&g_output_msg[buf_idx], &roll, 4); 	buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &pitch, 4); 	buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &yaw, 4); 	buf_idx += 4;

	uint16_t payload_size = buf_idx - 6;
	memcpy(&g_output_msg[4], &payload_size, 2); // 2-byte checksum
	memset(&g_output_msg[buf_idx], 0, 2); // 2-byte checksum, no use

	platform_uart_send(UART_PORT1, g_output_msg, buf_idx + 2);
	platform_uart_send(UART_PORT2, g_output_msg, buf_idx + 2);
	platform_uart_send(UART_PORT3, g_output_msg, buf_idx + 2);
}

static void handle_external_message(uint8_t *data, size_t size) {
	send_control_command(g_rc_values[3], g_rc_values[2], g_rc_values[0]);
}

void remote_control_setup(void) {
	subscribe(EXTERNAL_MESSAGE, on_external_message);
	subscribe(SCHEDULER_100HZ, handle_external_message);
}
