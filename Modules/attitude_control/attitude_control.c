#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>

static vector3d_t g_euler_angle;
static int g_ctl_roll = 0;
static int g_ctl_pitch = 0;
static int g_ctl_yaw = 0;
static int g_ctl_alt = 0;

static void attitude_update(uint8_t *data, size_t size) {
	g_euler_angle = *(vector3d_t*)data;
}

static void orientation_control_update(uint8_t *data, size_t size) {
    g_ctl_roll 	= *(int*)&data[0];
    g_ctl_pitch	= *(int*)&data[4];
    g_ctl_yaw 	= *(int*)&data[8];
    g_ctl_alt 	= *(int*)&data[12];
}

static void ac_init(void) {

}

static void ac_loop(uint8_t *data, size_t size) {

}

void attitude_control_setup(void) {
	ac_init();
	subscribe(SENSOR_ATTITUDE_ANGLE, attitude_update);
	subscribe(COMMAND_SET_TARGET_ORIENTATION, orientation_control_update);
	subscribe(SCHEDULER_1KHZ, ac_loop);
}
