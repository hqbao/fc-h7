#include "nav_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
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

static state_t g_state = DISARMED;
static rc_att_ctl_t g_rc_att_ctl;

static pid_control_t g_pid_nav_x;
static pid_control_t g_pid_nav_y;
static pid_control_t g_pid_nav_z;

static vector3d_t g_pos_final = {0, 0, 0};
static vector3d_t g_pos_target = {0, 0, 0};

static double g_yaw_veloc = 0;

uint8_t g_target_data[32] = {0};

static void move_in_control_update(uint8_t *data, size_t size) {
	g_rc_att_ctl.roll 	= (*(float*)&data[0]);
	g_rc_att_ctl.pitch	= (*(float*)&data[4]);
	g_rc_att_ctl.yaw 	= (*(float*)&data[8]);
	g_rc_att_ctl.alt 	= (*(float*)&data[12]);
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (g_rc_att_ctl.roll != 0) {
		g_pos_target.y = g_pos_final.y + g_rc_att_ctl.roll * 0.04;
	}

	if (g_rc_att_ctl.pitch != 0) {
		g_pos_target.x = g_pos_final.x + g_rc_att_ctl.pitch * 0.04;
	}

	if (g_rc_att_ctl.alt != 0) {
		g_pos_target.z = g_pos_final.z + g_rc_att_ctl.alt * 0.04;
	}

	g_yaw_veloc = 3.0 * g_rc_att_ctl.yaw / NAV_FREQ;
}

static void pid_setup(void) {
	pid_control_init(&g_pid_nav_x);
	pid_control_set_p_gain(&g_pid_nav_x, 0);
	pid_control_set_d_gain(&g_pid_nav_x, 0);
	pid_control_set_i_gain(&g_pid_nav_x, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_x, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_y);
	pid_control_set_p_gain(&g_pid_nav_y, 0);
	pid_control_set_d_gain(&g_pid_nav_y, 0);
	pid_control_set_i_gain(&g_pid_nav_y, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_y, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_z);
	pid_control_set_p_gain(&g_pid_nav_z, 500);
	pid_control_set_d_gain(&g_pid_nav_z, 100);
	pid_control_set_i_gain(&g_pid_nav_z, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_z, 1.0, 1.0, 1.0);
}

static void nav_control_loop(void) {
	double dt = 1.0 / NAV_FREQ;
	pid_control_update(&g_pid_nav_x, g_pos_final.x, g_pos_target.x, dt);
	pid_control_update(&g_pid_nav_y, g_pos_final.y, g_pos_target.y, dt);
	pid_control_update(&g_pid_nav_z, g_pos_final.z, g_pos_target.z, dt);

	memcpy(&g_target_data[0], 	&g_pid_nav_y.output, 8);
	memcpy(&g_target_data[8],	&g_pid_nav_x.output, 8);
	memcpy(&g_target_data[16], 	&g_yaw_veloc, 8);
	memcpy(&g_target_data[24], 	&g_pid_nav_z.output, 8);
	publish(COMMAND_SET_TARGET_ORIENTATION, (uint8_t*)g_target_data, 32);
}

static void position_update(uint8_t *data, size_t size) {
	memcpy(&g_pos_final, data, size);
	nav_control_loop();
}

static void reset(void) {
	pid_control_reset(&g_pid_nav_x, g_pos_final.x);
	pid_control_reset(&g_pid_nav_y, g_pos_final.y);
	pid_control_reset(&g_pid_nav_z, g_pos_final.z);
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		reset();
	}
}

void nav_control_setup(void) {
	pid_setup();
	subscribe(NAV_POSITION_UPDATE, position_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(SCHEDULER_100HZ, loop_100hz);
}
