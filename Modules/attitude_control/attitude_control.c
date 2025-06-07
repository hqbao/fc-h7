#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>

#define INIT_SPEED 200
#define PID_FREQ 1000

typedef enum {
	INIT = 0,
	READY,
	TAKE_OFF,
	FLYING,
} state_t;

static vector3d_t g_euler_angle;
static float g_air_pressure_alt = 0;

static float g_ctl_roll = 0;
static float g_ctl_pitch = 0;
static float g_ctl_yaw = 0;
static float g_ctl_alt = 0;
static float g_ctl_state = 0;
static float g_ctl_mode = 0;

static int g_pwm_duty[4] = {100, 100, 100, 100};

static state_t g_state = INIT;

static void attitude_angle_update(uint8_t *data, size_t size) {
	g_euler_angle = *(vector3d_t*)data;
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(float*)data;
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_state = READY;
	else publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}

static void move_in_control_update(uint8_t *data, size_t size) {
    g_ctl_roll 	= (float)(*(int*)&data[0]) / 10;
    g_ctl_pitch	= (float)(*(int*)&data[4]) / 10;
    g_ctl_yaw 	= (float)(*(int*)&data[8]) / 10;
    g_ctl_alt 	= (float)(*(int*)&data[12]) / 10;
    g_ctl_state = data[16];
    g_ctl_mode 	= data[17];
}

static void attitude_control_init(void) {
	platform_pwm_init(PWM_PORT1);
	platform_pwm_init(PWM_PORT2);
	platform_pwm_init(PWM_PORT3);
	platform_pwm_init(PWM_PORT4);
}

static void attitude_control_loop(uint8_t *data, size_t size) {
	if (g_state == INIT) {
		platform_pwm_send(PWM_PORT1, 0);
		platform_pwm_send(PWM_PORT2, 0);
		platform_pwm_send(PWM_PORT3, 0);
		platform_pwm_send(PWM_PORT4, 0);
	}
	else if (g_state == READY) {
		platform_pwm_send(PWM_PORT1, INIT_SPEED);
		platform_pwm_send(PWM_PORT2, INIT_SPEED);
		platform_pwm_send(PWM_PORT3, INIT_SPEED);
		platform_pwm_send(PWM_PORT4, INIT_SPEED);
	}
	else if (g_state == TAKE_OFF) {
		platform_pwm_send(PWM_PORT1, g_pwm_duty[0]);
		platform_pwm_send(PWM_PORT2, g_pwm_duty[1]);
		platform_pwm_send(PWM_PORT3, g_pwm_duty[2]);
		platform_pwm_send(PWM_PORT4, g_pwm_duty[3]);
	}
}

static void attitude_control_loop_slow(uint8_t *data, size_t size) {
	static uint8_t g_output_msg[128] = {'d', 'b', 0x02 /* Info */, 0x01 /* State */};
	int buf_idx = 6;
	memcpy(&g_output_msg[buf_idx], &g_euler_angle.x, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &g_euler_angle.y, 4); buf_idx += 4;
	memcpy(&g_output_msg[buf_idx], &g_euler_angle.z, 4); buf_idx += 4;

	uint16_t payload_size = buf_idx - 6;
	memcpy(&g_output_msg[4], &payload_size, 2); // 2-byte checksum
	memset(&g_output_msg[buf_idx], 0, 2); // 2-byte checksum, no use

	platform_uart_send(UART_PORT1, g_output_msg, buf_idx + 2);

	if (g_state == INIT) {
		platform_toggle_led(0);
	}
}

void attitude_control_setup(void) {
	attitude_control_init();
	subscribe(NOTIFY_IMU_CALIBRATION_RESULT, on_imu_calibration_result);
	subscribe(SENSOR_ATTITUDE_ANGLE, attitude_angle_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);
	subscribe(SCHEDULER_25HZ, attitude_control_loop_slow);

	publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}
