#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define MIN_SPEED 100
#define MAX_SPEED 2000
#define PID_FREQ 1000

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKE_OFF,
	FLYING,
	TESTING,
} state_t;

typedef struct {
	float state;
	float mode;
} rc_state_ctl_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

static int g_output_speed[4] = {0, 0, 0, 0};
static vector3d_t g_state_vector = {0, 0, 1};
static vector3d_t g_target_vector = {0, 0, 1};
static state_t g_state = DISARMED;
static char g_imu_calibrated = 0;
static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static rc_att_ctl_t g_rc_att_ctl;

static pid_control_t g_pid_att_roll;
static pid_control_t g_pid_att_pitch;
static pid_control_t g_pid_att_yaw;
static pid_control_t g_pid_alt;

static void attitude_update(uint8_t *data, size_t size) {
	g_state_vector = *(vector3d_t*)data;
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_imu_calibrated = 1;
	else publish(SENSOR_IMU_CALIBRATE_GYRO, NULL, 0);
}

static void state_control_update(uint8_t *data, size_t size) {
	g_rc_state_ctl.state = data[0];
	g_rc_state_ctl.mode = data[1];
}

static void move_in_control_update(uint8_t *data, size_t size) {
	g_rc_att_ctl.roll 	= (*(float*)&data[0]);
	g_rc_att_ctl.pitch	= (*(float*)&data[4]);
	g_rc_att_ctl.yaw 	= (*(float*)&data[8]);
	g_rc_att_ctl.alt 	= (*(float*)&data[12]);
}

static void update_target_attitude(uint8_t *data, size_t size) {
	g_target_vector = *(vector3d_t*)data;
}

static void pid_setup(void) {
	pid_control_init(&g_pid_att_roll, PID_FREQ);
	pid_control_set_p_gain(&g_pid_att_roll, 50);
	pid_control_set_d_gain(&g_pid_att_roll, 5);
	pid_control_set_i_gain(&g_pid_att_roll, 1.0, 1.0);
	pid_control_set_smooth(&g_pid_att_roll, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_att_pitch, PID_FREQ);
	pid_control_set_p_gain(&g_pid_att_pitch, 50);
	pid_control_set_d_gain(&g_pid_att_pitch, 5);
	pid_control_set_i_gain(&g_pid_att_pitch, 1.0, 1.0);
	pid_control_set_smooth(&g_pid_att_pitch, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_att_yaw, PID_FREQ);
	pid_control_set_p_gain(&g_pid_att_yaw, 50);
	pid_control_set_d_gain(&g_pid_att_yaw, 5);
	pid_control_set_i_gain(&g_pid_att_yaw, 1.0, 1.0);
	pid_control_set_smooth(&g_pid_att_yaw, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_alt, PID_FREQ);
	pid_control_set_p_gain(&g_pid_alt, 50);
	pid_control_set_d_gain(&g_pid_alt, 5);
	pid_control_set_i_gain(&g_pid_alt, 1.0, 1.0);
	pid_control_set_smooth(&g_pid_alt, 1.0, 1.0, 1.0);
}

static void pid_loop(void) {

}

static void attitude_control_loop(uint8_t *data, size_t size) {
	pid_loop();

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
		g_output_speed[0] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw * 20, 0, MAX_SPEED);
		g_output_speed[1] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt * 20, 0, MAX_SPEED);
		g_output_speed[2] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll * 20, 0, MAX_SPEED);
		g_output_speed[3] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch * 20, 0, MAX_SPEED);
		platform_dshot_send(DSHOT_PORT1, g_output_speed[0]);
		platform_dshot_send(DSHOT_PORT2, g_output_speed[1]);
		platform_dshot_send(DSHOT_PORT3, g_output_speed[2]);
		platform_dshot_send(DSHOT_PORT4, g_output_speed[3]);
	}
}

static void attitude_control_loop_100hz(uint8_t *data, size_t size) {
	if (g_rc_state_ctl.state == 0) {
		g_state = DISARMED;
	}

	if (g_rc_state_ctl.state == 1 && g_rc_state_ctl_prev.state == 0) {
		if (g_imu_calibrated == 1) {
			g_state = ARMED;
		}
	}

	if (g_state == ARMED) {
		char stick1_most_left 	= g_rc_att_ctl.yaw == -49;
		char stich1_most_bottom = g_rc_att_ctl.alt == -49;
		char stick2_most_right 	= g_rc_att_ctl.roll == 49;
		char stich2_most_bottom = g_rc_att_ctl.pitch == -49;
		if (stick1_most_left && stich1_most_bottom &&
				stick2_most_right && stich2_most_bottom) {
			g_state = READY;
		}
	}

	if (g_state == READY) {
		if (g_rc_att_ctl.alt > 5) {
			g_state = g_rc_state_ctl.state == 1 ? TAKE_OFF : TESTING;
		}
	}

	memcpy(&g_rc_state_ctl_prev, &g_rc_state_ctl, sizeof(rc_state_ctl_t));
}

void attitude_control_setup(void) {
	pid_setup();

	platform_dshot_init(DSHOT_PORT1);
	platform_dshot_init(DSHOT_PORT2);
	platform_dshot_init(DSHOT_PORT3);
	platform_dshot_init(DSHOT_PORT4);

	subscribe(SENSOR_IMU_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(SENSOR_ATTITUDE_VECTOR, attitude_update);
	subscribe(COMMAND_SET_STATE, state_control_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(NAV_SET_TARGET_ATTITUDE, update_target_attitude);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);
	subscribe(SCHEDULER_100HZ, attitude_control_loop_100hz);

	publish(SENSOR_IMU_CALIBRATE_GYRO, NULL, 0);
}
