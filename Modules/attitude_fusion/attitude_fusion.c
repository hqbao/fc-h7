#include "attitude_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <filter1.h>

#define MAX_IMU_ACCEL 16384
#define SSF_GYRO (16.4)
#define DEG2RAD 0.01745329251
#define IMU_FREQ 8000

static int16_t g_imu_gyro[3] = {0, 0, 0};
static int16_t g_imu_accel[3] = {0, 0, MAX_IMU_ACCEL};
static filter1_t g_f1;

vector3d_t g_pred_vector = {0, 0, 1};
vector3d_t g_pred_angle = {0, 0, 1};

static void fusion_update_gyro(void) {
	float dt = 1.0 / IMU_FREQ;
	float gx = g_imu_gyro[0] / SSF_GYRO * DEG2RAD * dt;
	float gy = g_imu_gyro[1] / SSF_GYRO * DEG2RAD * dt;
	float gz = g_imu_gyro[2] / SSF_GYRO * DEG2RAD * dt;
	filter1_predict(&g_f1, gx, gy, gz);

	g_pred_vector.x = g_f1.v_pred.y;
	g_pred_vector.y = g_f1.v_pred.x;
	g_pred_vector.z = g_f1.v_pred.z;
	publish(SENSOR_ATTITUDE_VECTOR, (uint8_t*)&g_pred_vector, sizeof(vector3d_t));

	g_pred_angle.x = g_f1.pred_euler_angle.y;
	g_pred_angle.y = g_f1.pred_euler_angle.x;
	g_pred_angle.z = g_f1.pred_euler_angle.z;
	publish(SENSOR_ATTITUDE_ANGLE, (uint8_t*)&g_pred_angle, sizeof(vector3d_t));
}

static void fusion_update_accel(void) {
	filter1_update(&g_f1,
			g_imu_accel[1],
			g_imu_accel[0],
			g_imu_accel[2]);
}

static void gyro_update(uint8_t *data, size_t size) {
	memcpy(g_imu_gyro, data, size);
	fusion_update_gyro();
}

static void accel_update(uint8_t *data, size_t size) {
	memcpy(g_imu_accel, data, size);
	fusion_update_accel();
}

static void init(void) {
	filter1_init(&g_f1, 1.0 / IMU_FREQ);
	g_f1.no_correction = 1;
}

void attitude_fusion_setup(void) {
	init();
	subscribe(SENSOR_IMU_GYRO, gyro_update);
	subscribe(SENSOR_IMU_ACCEL, accel_update);
	publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}

