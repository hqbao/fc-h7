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
static vector3d_t g_linear_veloc_zero = {0, 0, 0};
static vector3d_t g_local_pos = {0, 0, 0};
static vector3d_t g_pos = {0, 0, 0};
static vector3d_t g_pos_bias = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static double coef11 = 1.0;
static double coef12 = 0.01;
static double coef23 = 1000.0;
static double coef24 = 1000.0;
static double coef33 = 0.25;
static double coef34 = 0.125;
static double coef43 = 0.5;
static double coef44 = 100;
static double coef51 = 1.0;
static double coef52 = 0.25;
static double scale11 = 1000;
static double scale12 = 10000;
static double scale21 = 5.0;
static double scale22 = 100;
static double scale3 = 0.01;

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(double*)data;
	double air_pressure_alt_d = g_air_pressure_alt - g_air_pressure_alt_prev;
	g_air_pressure_alt_prev = g_air_pressure_alt;
	if (fabs(g_linear_accel.z) > 0.00015)
		air_pressure_alt_d += scale12 * fabs(g_linear_accel.z);
	g_local_pos.z += air_pressure_alt_d;
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	g_optflow.x = (double)(*(int*)&data[4]) * 0.1;
	g_optflow.y = (double)(*(int*)&data[0]) * 0.1;
	g_optflow_x += g_optflow.x;
	g_optflow_y += g_optflow.y;
	g_local_pos.x += g_optflow.x;
	g_local_pos.y += g_optflow.y;
	g_local_pos.x += g_optflow.x * scale11 * fabs(g_linear_accel.x);
	g_local_pos.y += g_optflow.y * scale11 * fabs(g_linear_accel.y);
}

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	g_linear_accel.x = coef11 * v.x;
	g_linear_accel.y = coef11 * v.y;
	g_linear_accel.z = coef12 * v.z;

	g_linear_veloc_zero.x += coef23 / NAV_FREQ * g_linear_accel.x;
	g_linear_veloc_zero.y += coef23 / NAV_FREQ * g_linear_accel.y;
	g_linear_veloc_zero.z += coef24 / NAV_FREQ * g_linear_accel.z;

	g_linear_veloc_zero.x += coef33 / NAV_FREQ * (-g_linear_veloc_zero.x);
	g_linear_veloc_zero.y += coef33 / NAV_FREQ * (-g_linear_veloc_zero.y);
	g_linear_veloc_zero.z += coef34 / NAV_FREQ * (-g_linear_veloc_zero.z);

	g_local_pos.x += scale21 / NAV_FREQ * (g_optflow_x - g_local_pos.x);
	g_local_pos.y += scale21 / NAV_FREQ * (g_optflow_y - g_local_pos.y);
	g_local_pos.z += scale22 / NAV_FREQ * (g_air_pressure_alt - g_local_pos.z);

	g_pos.x += coef43 / NAV_FREQ * g_linear_veloc_zero.x;
	g_pos.y += coef43 / NAV_FREQ * g_linear_veloc_zero.y;
	g_pos.z += coef44 / NAV_FREQ * g_linear_veloc_zero.z;

	g_pos.x += coef51 / NAV_FREQ * (g_local_pos.x - g_pos.x);
	g_pos.y += coef51 / NAV_FREQ * (g_local_pos.y - g_pos.y);
	g_pos.z += coef52 / NAV_FREQ * (g_local_pos.z - g_pos.z);

	int number1 = g_optflow_x;
	int number2 = g_local_pos.x;
	int number3 = g_pos.x;
	static uint8_t g_msg[16] = {0};
	memcpy(&g_msg[0], &number1, 4);
	memcpy(&g_msg[4], &number2, 4);
	memcpy(&g_msg[8], &number3, 4);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}

static void loop_1khz(uint8_t *data, size_t size) {
	vector3d_sub(&g_pos_final, &g_pos, &g_pos_bias);
	vector3d_scale(&g_pos_final, &g_pos_final, scale3);
	publish(NAV_POSITION_UPDATE, (uint8_t*)&g_pos_final, sizeof(vector3d_t));
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		vector3d_set(&g_pos_bias, &g_pos);
	}
}

void nav_fusion_setup(void) {
	subscribe(SCHEDULER_1KHZ, loop_1khz);
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
}

