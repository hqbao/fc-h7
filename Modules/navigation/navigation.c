#include "navigation.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

static double g_air_pressure_alt = 0;
static vector3d_t g_linear_veloc = {0, 0, 0};
static rc_att_ctl_t g_rc_att_ctl;

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	vector3d_add(&g_linear_veloc, &g_linear_veloc, &v);
	publish(SENSOR_LINEAR_VELOC, (uint8_t*)&g_linear_veloc, sizeof(vector3d_t));
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(float*)data;
}

static void move_in_control_update(uint8_t *data, size_t size) {
	g_rc_att_ctl.roll 	= (*(float*)&data[0]);
	g_rc_att_ctl.pitch	= (*(float*)&data[4]);
	g_rc_att_ctl.yaw 	= (*(float*)&data[8]);
	g_rc_att_ctl.alt 	= (*(float*)&data[12]);
}

void navigation_setup(void) {
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
}

