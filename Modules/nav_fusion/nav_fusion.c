#include "nav_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <pid_control.h>
#include <string.h>
#include <math.h>
#include <macro.h>

#define NAV_FREQ 1000

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKING_OFF,
	FLYING,
	TESTING,
} state_t;

static state_t g_state = DISARMED;
static double g_optflow_x = 0;
static double g_optflow_y = 0;
static double g_air_pressure_alt = 0;
static double g_air_pressure_alt_prev = 0;
static vector3d_t g_optflow = {0, 0, 0};
static vector3d_t g_linear_accel = {0, 0, 0};
static vector3d_t g_linear_veloc = {0, 0, 0};
static vector3d_t g_linear_veloc_bias = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};

static vector3d_t g_local_pos = {0, 0, 0};
static vector3d_t g_pos = {0, 0, 0};
static vector3d_t g_pos_bias = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static double coef1 = 1000;
static double coef21 = 25;
static double coef22 = 10;
static double coef31 = 0.025;
static double coef32 = 0.025;
static double coef41 = 20;
//static double coef42 = 1;

static double scale11 = 0.1;
static double scale12 = 0.025;
static double scale2 = 0.1;
static double scale3 = 1;
static double threshold3 = 20;

static void optflow_sensor_update(uint8_t *data, size_t size) {
	g_optflow.x = (double)(*(int*)&data[4]) * scale3;
	g_optflow.y = (double)(*(int*)&data[0]) * scale3;

	g_optflow_x += g_optflow.x * coef31;
	g_optflow_y += g_optflow.y * coef31;

	g_local_pos.x = g_optflow_x;
	g_local_pos.y = g_optflow_y;

	g_linear_veloc.x += coef31 * (g_optflow.x - g_linear_veloc.x);
	g_linear_veloc.y += coef31 * (g_optflow.y - g_linear_veloc.y);
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(double*)data;
	if (fabs(g_linear_accel.z) > threshold3) {
		g_local_pos.z += scale11 * (g_air_pressure_alt - g_local_pos.z);
	} else {
		g_local_pos.z += scale12 * (g_air_pressure_alt - g_local_pos.z);
	}

	double g_air_pressure_alt_d = g_local_pos.z - g_air_pressure_alt_prev;
	g_air_pressure_alt_prev = g_local_pos.z;
	g_linear_veloc.z += coef32 * (g_air_pressure_alt_d - g_linear_veloc.z);
}

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	g_linear_accel.x = coef1 * v.x;
	g_linear_accel.y = coef1 * v.y;
	g_linear_accel.z = coef1 * v.z;

	g_linear_veloc.x += 1.0 / NAV_FREQ * g_linear_accel.x;
	g_linear_veloc.y += 1.0 / NAV_FREQ * g_linear_accel.y;
	g_linear_veloc.z += 1.0 / NAV_FREQ * g_linear_accel.z;

	g_pos.x += coef41 / NAV_FREQ * g_linear_veloc.x;
	g_pos.y += coef41 / NAV_FREQ * g_linear_veloc.y;
	//g_pos.z += coef42 / NAV_FREQ * g_linear_veloc.z;

	g_pos.x += coef21 / NAV_FREQ * (g_local_pos.x - g_pos.x);
	g_pos.y += coef21 / NAV_FREQ * (g_local_pos.y - g_pos.y);
	g_pos.z += coef22 / NAV_FREQ * (g_local_pos.z - g_pos.z);
}

static void loop_1khz(uint8_t *data, size_t size) {
	vector3d_sub(&g_pos_final, &g_pos, &g_pos_bias);
	vector3d_scale(&g_pos_final, &g_pos_final, scale2);
	g_pos_final.x = -g_pos_final.x;
	g_pos_final.y = -g_pos_final.y;

	vector3d_sub(&g_linear_veloc_final, &g_linear_veloc, &g_linear_veloc_bias);
	g_linear_veloc_final.x = -g_linear_veloc_final.x;
	g_linear_veloc_final.y = -g_linear_veloc_final.y;

	static uint8_t g_nav_pos_msg[sizeof(vector3d_t) * 2] = {0};
	memcpy(g_nav_pos_msg, &g_pos_final, sizeof(vector3d_t));
	memcpy(&g_nav_pos_msg[sizeof(vector3d_t)], &g_linear_veloc_final, sizeof(vector3d_t));

	publish(NAV_POSITION_UPDATE, (uint8_t*)&g_nav_pos_msg, sizeof(vector3d_t) * 2);

	int number1 = g_air_pressure_alt;
	int number2 = g_local_pos.z;
	int number3 = g_pos.z;
	static uint8_t g_msg[16] = {0};
	memcpy(&g_msg[0], &number1, 4);
	memcpy(&g_msg[4], &number2, 4);
	memcpy(&g_msg[8], &number3, 4);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		vector3d_set(&g_pos_bias, &g_pos);
		vector3d_set(&g_linear_veloc_bias, &g_linear_veloc);
	}
}

void nav_fusion_setup(void) {
	subscribe(SCHEDULER_1KHZ, loop_1khz);
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
}

