#include "nav_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define NAV_FREQ 1000

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

static rc_state_ctl_t g_rc_state_ctl;
static rc_att_ctl_t g_rc_att_ctl;

static pid_control_t g_pid_nav_x;
static pid_control_t g_pid_nav_y;
static pid_control_t g_pid_nav_z;

static vector3d_t g_pos_final = {0, 0, 0};
static vector3d_t g_pos_target = {0, 0, 0};

static double g_yaw_veloc = 0;

uint8_t g_target_data[32] = {0};

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

static void pid_setup(void) {
	pid_control_init(&g_pid_nav_x);
	pid_control_set_p_gain(&g_pid_nav_x, 1);
	pid_control_set_d_gain(&g_pid_nav_x, 1);
	pid_control_set_i_gain(&g_pid_nav_x, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_x, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_y);
	pid_control_set_p_gain(&g_pid_nav_y, 1);
	pid_control_set_d_gain(&g_pid_nav_y, 1);
	pid_control_set_i_gain(&g_pid_nav_y, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_y, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_z);
	pid_control_set_p_gain(&g_pid_nav_z, 1);
	pid_control_set_d_gain(&g_pid_nav_z, 1);
	pid_control_set_i_gain(&g_pid_nav_z, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_z, 1.0, 1.0, 1.0);
}

static void nav_control_loop(void) {
	g_pos_target.x = g_pos_final.x + g_rc_att_ctl.pitch * 0.3;
	g_pos_target.y = g_pos_final.y + g_rc_att_ctl.roll * 0.3;
	g_pos_target.z = g_pos_final.z + g_rc_att_ctl.alt * 0.3;
	g_yaw_veloc = 0.1 * g_rc_att_ctl.yaw / NAV_FREQ;

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

void nav_control_setup(void) {
	pid_setup();
	subscribe(NAV_POSITION_UPDATE, position_update);
	subscribe(COMMAND_SET_STATE, state_control_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
}
