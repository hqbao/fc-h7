#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <macro.h>

#define MIN_SPEED 100
#define MAX_SPEED 2000
#define PID_FREQ 1000
#define IMU_FREQ 8000

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKE_OFF,
	FLYING,
	TESTING,
} state_t;

static vector3d_t g_euler_angle = {0, 0, 90};

static float g_ctl_roll = 0;
static float g_ctl_pitch = 0;
static float g_ctl_yaw = 0;
static float g_ctl_alt = 0;
static float g_ctl_state = 0;
static float g_ctl_state_prev = 0;
static float g_ctl_mode = 0;
static float g_ctl_mode_prev = 0;

static int g_output_speed[4] = {0, 0, 0, 0};

static state_t g_state = DISARMED;
static char g_imu_calibrated = 0;

static void attitude_angle_update(uint8_t *data, size_t size) {
	g_euler_angle = *(vector3d_t*)data;
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_imu_calibrated = 1;
	else publish(SENSOR_IMU_CALIBRATE_GYRO, NULL, 0);
}

static void move_in_control_update(uint8_t *data, size_t size) {
    g_ctl_roll 	= (*(float*)&data[0]);
    g_ctl_pitch	= (*(float*)&data[4]);
    g_ctl_yaw 	= (*(float*)&data[8]);
    g_ctl_alt 	= (*(float*)&data[12]);
    g_ctl_state = data[16];
    g_ctl_mode 	= data[17];
}

static void attitude_control_init(void) {
	platform_dshot_init(DSHOT_PORT1);
	platform_dshot_init(DSHOT_PORT2);
	platform_dshot_init(DSHOT_PORT3);
	platform_dshot_init(DSHOT_PORT4);
}

static void attitude_control_loop(uint8_t *data, size_t size) {
	if (g_state == DISARMED) {
		platform_dshot_send(DSHOT_PORT1, 0);
		platform_dshot_send(DSHOT_PORT2, 0);
		platform_dshot_send(DSHOT_PORT3, 0);
		platform_dshot_send(DSHOT_PORT4, 0);
	}
	if (g_state == ARMED) {
		platform_dshot_send(DSHOT_PORT1, 0);
		platform_dshot_send(DSHOT_PORT2, 0);
		platform_dshot_send(DSHOT_PORT3, 0);
		platform_dshot_send(DSHOT_PORT4, 0);
	}
	else if (g_state == READY) {
		platform_dshot_send(DSHOT_PORT1, MIN_SPEED);
		platform_dshot_send(DSHOT_PORT2, MIN_SPEED);
		platform_dshot_send(DSHOT_PORT3, MIN_SPEED);
		platform_dshot_send(DSHOT_PORT4, MIN_SPEED);
	}
	else if (g_state == TAKE_OFF) {
		platform_dshot_send(DSHOT_PORT1, MIN_SPEED + g_output_speed[0]);
		platform_dshot_send(DSHOT_PORT2, MIN_SPEED + g_output_speed[1]);
		platform_dshot_send(DSHOT_PORT3, MIN_SPEED + g_output_speed[2]);
		platform_dshot_send(DSHOT_PORT4, MIN_SPEED + g_output_speed[3]);
	}
	else if (g_state == TESTING) {
		g_output_speed[0] = LIMIT(MIN_SPEED + g_ctl_yaw * 20, 0, MAX_SPEED);
		g_output_speed[1] = LIMIT(MIN_SPEED + g_ctl_alt * 20, 0, MAX_SPEED);
		g_output_speed[2] = LIMIT(MIN_SPEED + g_ctl_roll * 20, 0, MAX_SPEED);
		g_output_speed[3] = LIMIT(MIN_SPEED + g_ctl_pitch * 20, 0, MAX_SPEED);
		platform_dshot_send(DSHOT_PORT1, g_output_speed[0]);
		platform_dshot_send(DSHOT_PORT2, g_output_speed[1]);
		platform_dshot_send(DSHOT_PORT3, g_output_speed[2]);
		platform_dshot_send(DSHOT_PORT4, g_output_speed[3]);
	}
}

static void attitude_control_loop_100hz(uint8_t *data, size_t size) {
	if (g_ctl_state == 0) {
		g_state = DISARMED;
	}

	if (g_ctl_state == 1 && g_ctl_state_prev == 0) {
		if (g_imu_calibrated == 1) {
			g_state = ARMED;
		}
	}

	if (g_state == ARMED) {
		char stick1_most_left 	= g_ctl_yaw == -49;
		char stich1_most_bottom = g_ctl_alt == -49;
		char stick2_most_right 	= g_ctl_roll == 49;
		char stich2_most_bottom = g_ctl_pitch == -49;
		if (stick1_most_left && stich1_most_bottom &&
				stick2_most_right && stich2_most_bottom) {
			g_state = READY;
		}
	}

	if (g_state == READY) {
		if (g_ctl_alt > 5) {
			g_state = g_ctl_state == 1 ? TAKE_OFF : TESTING;
		}
	}

	g_ctl_state_prev = g_ctl_state;
	g_ctl_mode_prev = g_ctl_mode;
}

static void attitude_control_loop_25hz(uint8_t *data, size_t size) {
	static uint8_t g_output_msg[128] = {'d', 'b', 0x00 /* Info */, 0x05 /* Linear velocity */};
	int buf_idx = 6;

	int x = (int)g_euler_angle.x;
	int y = (int)g_euler_angle.y;
	int z = (int)g_euler_angle.z;
	memcpy(&g_output_msg[buf_idx], &x, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &y, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &z, 4); buf_idx += 4;

	uint16_t payload_size = buf_idx - 6;
	memcpy(&g_output_msg[4], &payload_size, 2); // 2-byte checksum
	memset(&g_output_msg[buf_idx], 0, 2); // 2-byte checksum, no use

	platform_uart_send(UART_PORT1, g_output_msg, buf_idx + 2);

	if (g_imu_calibrated == 0) {
		platform_toggle_led(0);
	}
}

void attitude_control_setup(void) {
	attitude_control_init();

	subscribe(SENSOR_IMU_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(SENSOR_ATTITUDE_ANGLE, attitude_angle_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);
	subscribe(SCHEDULER_100HZ, attitude_control_loop_100hz);
	subscribe(SCHEDULER_25HZ, attitude_control_loop_25hz);

	publish(SENSOR_IMU_CALIBRATE_GYRO, NULL, 0);
}
