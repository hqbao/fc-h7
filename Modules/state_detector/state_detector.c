#include "state_detector.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <vector3d.h>
#include <macro.h>

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKING_OFF,
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

static vector3d_t g_optflow = {0, 0, 0};
static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static rc_att_ctl_t g_rc_att_ctl;
static state_t g_state = DISARMED;
static state_t g_state_prev = DISARMED;
static char g_imu_calibrated = 0;

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

static void optflow_sensor_update(uint8_t *data, size_t size) {
	g_optflow.x = (double)(*(int*)&data[0]);
	g_optflow.y = (double)(*(int*)&data[4]);
	g_optflow.z = (double)(*(int*)&data[8]);
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_imu_calibrated = 1;
	else publish(SENSOR_IMU1_CALIBRATE_GYRO, NULL, 0);
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (g_rc_state_ctl.state == 0) {
		g_state = DISARMED;
	}

	if (g_rc_state_ctl.state == 1 && g_rc_state_ctl_prev.state == 0) {
		if (g_imu_calibrated == 1) {
			g_state = ARMED;
		}
	}

	if (g_state == ARMED) {
		char stick1_most_left 	= g_rc_att_ctl.yaw == -90;
		char stich1_most_bottom = g_rc_att_ctl.alt == -90;
		char stick2_most_right 	= g_rc_att_ctl.roll == 90;
		char stich2_most_bottom = g_rc_att_ctl.pitch == -90;
		if (stick1_most_left && stich1_most_bottom &&
				stick2_most_right && stich2_most_bottom) {
			g_state = READY;
		}
	}

	if (g_state == READY) {
		if (g_rc_att_ctl.alt > 5) {
			g_state = g_rc_state_ctl.state == 1 ? TAKING_OFF : TESTING;
		}
	}

	if (g_state == TAKING_OFF) {
		if (g_optflow.z > 100) {
			g_state = FLYING;
		}
	}

	memcpy(&g_rc_state_ctl_prev, &g_rc_state_ctl, sizeof(rc_state_ctl_t));

	if (g_state != g_state_prev) {
		publish(STATE_DETECTION_UPDATE, (uint8_t*)&g_state, 1);
	}

	g_state_prev = g_state;
}

void state_detector_setup(void) {
	subscribe(SENSOR_IMU1_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(COMMAND_SET_STATE, state_control_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SCHEDULER_100HZ, loop_100hz);
	publish(SENSOR_IMU1_CALIBRATE_GYRO, NULL, 0);
}
