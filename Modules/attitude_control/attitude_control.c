#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>

static vector3d_t g_euler_angle;
static float g_air_pressure_alt = 0;

static int g_ctl_roll = 0;
static int g_ctl_pitch = 0;
static int g_ctl_yaw = 0;
static int g_ctl_alt = 0;

static void attitude_update(uint8_t *data, size_t size) {
	g_euler_angle = *(vector3d_t*)data;
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt = *(float*)data;
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] != 1) publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}

static void orientation_control_update(uint8_t *data, size_t size) {
    g_ctl_roll 	= *(int*)&data[0];
    g_ctl_pitch	= *(int*)&data[4];
    g_ctl_yaw 	= *(int*)&data[8];
    g_ctl_alt 	= *(int*)&data[12];
}

static void attitude_control_init(void) {

}

static void attitude_control_loop(uint8_t *data, size_t size) {

}

void attitude_control_setup(void) {
	attitude_control_init();
	subscribe(NOTIFY_IMU_CALIBRATION_RESULT, on_imu_calibration_result);
	subscribe(SENSOR_ATTITUDE_ANGLE, attitude_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(COMMAND_SET_TARGET_ORIENTATION, orientation_control_update);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);

	publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}
