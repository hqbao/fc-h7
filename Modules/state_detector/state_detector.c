#include "state_detector.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <vector3d.h>
#include <math.h>
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
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

static angle3d_t g_angular_state = {0, 0, 0};
static vector3d_t g_optflow = {0, 0, 0};
static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static rc_att_ctl_t g_rc_att_ctl;
static state_t g_state = DISARMED;
static state_t g_state_prev = DISARMED;
static char g_imu_calibrated = 0;

static void state_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_state_ctl, data, size);
}

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, size);
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

static void loop_1hz(uint8_t *data, size_t size) {
	static int counter_2s = 0;
	if (counter_2s <= 2) {
		if (counter_2s == 2) publish(SENSOR_IMU1_CALIBRATE_GYRO, NULL, 0);
		counter_2s++;
	}

	if (fabs(g_angular_state.roll) > 60 || fabs(g_angular_state.pitch) > 60) {
		g_state = DISARMED;
	}
}

static void angular_state_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_state, data, size);
}

void state_detector_setup(void) {
	subscribe(SENSOR_IMU1_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(COMMAND_SET_STATE, state_control_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SENSOR_ATTITUDE_ANGLE, angular_state_update);
	subscribe(SCHEDULER_100HZ, loop_100hz);
	subscribe(SCHEDULER_1HZ, loop_1hz);
}
