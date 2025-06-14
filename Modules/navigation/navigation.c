#include "navigation.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>

static float g_air_pressure_alt = 0;
static vector3d_t g_linear_veloc = {0, 0, 0};

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	vector3d_add(&g_linear_veloc, &g_linear_veloc, &v);
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(float*)data;
}

void navigation_setup(void) {
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
}

