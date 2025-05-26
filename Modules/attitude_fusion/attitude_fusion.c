#include "attitude_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <filter3.h>

#define MAX_IMU_ACCEL 16384
#define SSF_GYRO (32.8)
#define IMU_FREQ 1000

static int16_t g_imu_gyro[3] = {0, 0, 0};
static int16_t g_imu_accel[3] = {0, 0, MAX_IMU_ACCEL};
static filter3_t g_f3;

static void fusion_update_gyro(void) {
	float dt = 1.0 / IMU_FREQ;
	float gx = g_imu_gyro[0] / SSF_GYRO;
	float gy = g_imu_gyro[1] / SSF_GYRO;
	float gz = g_imu_gyro[2] / SSF_GYRO;
	filter3_predict(&g_f3, gx, gy, gz, dt);
	publish(SENSOR_ATTITUDE_VECTOR, (uint8_t*)&g_f3.pred_norm_accel, sizeof(vector3d_t));
	publish(SENSOR_ATTITUDE_ANGLE, (uint8_t*)&g_f3.pred_euler_angle, sizeof(vector3d_t));
}

static void fusion_update_accel(void) {
	filter3_update(&g_f3, g_imu_accel[0], g_imu_accel[1], g_imu_accel[2]);
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
	float gyro_noise = 0.000087; // rad/s
	float accel_noise = 10.0; // m/s^2
	filter3_init(&g_f3, gyro_noise, accel_noise);
}

void attitude_fusion_setup(void) {
	init();
	subscribe(SENSOR_IMU_GYRO, gyro_update);
	subscribe(SENSOR_IMU_ACCEL, accel_update);
}
