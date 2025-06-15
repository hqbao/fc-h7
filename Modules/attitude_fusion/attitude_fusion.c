#include "attitude_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <filter1.h>

#define MAX_IMU_ACCEL 16384
#define DEG2RAD 0.01745329251
#define IMU_FREQ 8000

#define ACCEL_OFFSET_X 0
#define ACCEL_OFFSET_Y 0
#define ACCEL_OFFSET_Z 0

static float g_imu_gyro[3] = {0, 0, 0};
static float g_imu_accel[3] = {0, 0, MAX_IMU_ACCEL};
static filter1_t g_f1;

vector3d_t g_pred_vector = {0, 0, 1};
vector3d_t g_linear_accel = {0, 0, 0};

vector3d_t g_accel_vector = {0, 0, MAX_IMU_ACCEL};
static float g_norm_accel = 1;

static void fusion_update_gyro(void) {
	double dt = 1.0 / IMU_FREQ;
	double gx = dt * g_imu_gyro[0] * DEG2RAD;
	double gy = dt * g_imu_gyro[1] * DEG2RAD;
	double gz = dt * g_imu_gyro[2] * DEG2RAD;
	filter1_predict(&g_f1, gx, gy, gz);

	g_pred_vector.x = g_f1.v_pred.y;
	g_pred_vector.y = g_f1.v_pred.x;
	g_pred_vector.z = g_f1.v_pred.z;
	publish(SENSOR_ATTITUDE_VECTOR, (uint8_t*)&g_pred_vector, sizeof(vector3d_t));
}

static void fusion_update_accel(void) {
	filter1_update(&g_f1,
			g_accel_vector.x,
			g_accel_vector.y,
			g_accel_vector.z);

	g_linear_accel.x = -g_f1.linear_acceleration.y / IMU_FREQ;
	g_linear_accel.y = -g_f1.linear_acceleration.x / IMU_FREQ;
	g_linear_accel.z = g_f1.linear_acceleration.z / IMU_FREQ;
	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_linear_accel, sizeof(vector3d_t));
}

static void gyro_update(uint8_t *data, size_t size) {
	memcpy(g_imu_gyro, data, size);
	fusion_update_gyro();
}

static void accel_update(uint8_t *data, size_t size) {
	g_imu_accel[1] = *(float*)&data[4] - ACCEL_OFFSET_X;
	g_imu_accel[0] = *(float*)&data[0] - ACCEL_OFFSET_Y;
	g_imu_accel[2] = *(float*)&data[8] - ACCEL_OFFSET_Z;
	vector3d_init(&g_accel_vector, g_imu_accel[1], g_imu_accel[0], g_imu_accel[2]);
	g_norm_accel = vector3d_norm(&g_accel_vector) / MAX_IMU_ACCEL;
	fusion_update_accel();
}

static void init(void) {
	filter1_init(&g_f1, 0.125 / IMU_FREQ);
	filter1_use_linear_acceleration(&g_f1, MAX_IMU_ACCEL);
	g_f1.no_correction = 1;
}

void attitude_fusion_setup(void) {
	init();
	subscribe(SENSOR_IMU_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_IMU_ACCEL_UPDATE, accel_update);
}

