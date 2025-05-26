#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include "icm42688p.h"

#define CALIBRATION_FREQ 1000 // SCHEDULER_4KHZ

typedef enum {
	init = 0,
	calibrating,
	ready,
} imu_mode_t;

static uint8_t g_i2c_buffer[32];
static int16_t g_imu_gyro_accel[6];
static int64_t g_imu_calibration[3];
static float g_imu_gyro_offset[3];
static int g_calibration_counter = 0;
static imu_mode_t g_imu_mode = init;

static void _i2c_read(uint8_t address, uint8_t *input, uint16_t input_size,
		uint8_t* output, uint16_t output_size) {
	platform_i2c_write_read_dma(I2C_PORT1, address, input, input_size, output, output_size);
}

static void _i2c_write(uint8_t address, uint8_t *data, uint16_t size) {
	platform_i2c_write(I2C_PORT1, address, data, size);
}

static void _icm42688p_init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR,
		uint8_t aMode, uint8_t gMode, bool CLKIN) {
	g_i2c_buffer[0] = ICM42688_INT_CONFIG1;
	g_i2c_buffer[1] = 0x00;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // select register bank 0

	g_i2c_buffer[0] = ICM42688_PWR_MGMT0;
	g_i2c_buffer[1] = gMode << 2 | aMode;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // set accel and gyro modes
	get_platform()->delay(1);

	g_i2c_buffer[0] = ICM42688_ACCEL_CONFIG0;
	g_i2c_buffer[1] = Ascale << 5 | AODR;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // set accel ODR and FS

	g_i2c_buffer[0] = ICM42688_GYRO_CONFIG0;
	g_i2c_buffer[1] = Gscale << 5 | GODR;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // set gyro ODR and FS

	g_i2c_buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
	g_i2c_buffer[1] = 0x44;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // set gyro and accel bandwidth to ODR/10

	// interrupt handling
	g_i2c_buffer[0] = ICM42688_INT_CONFIG;
	g_i2c_buffer[1] = 0x18 | 0x03;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // push-pull, pulsed, active HIGH interrupts

	g_i2c_buffer[0] = ICM42688_INT_CONFIG1;
	_i2c_read(ICM42688_ADDRESS, g_i2c_buffer, 1, &g_i2c_buffer[1], 1);     // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)

	g_i2c_buffer[0] = ICM42688_INT_CONFIG1;
	g_i2c_buffer[1] = g_i2c_buffer[1] & ~(0x10);
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)

	g_i2c_buffer[0] = ICM42688_INT_SOURCE0;
	g_i2c_buffer[1] = 0x08;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // data ready interrupt routed to INT1

	// Use external clock source
	if (CLKIN) {
		g_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
		g_i2c_buffer[1] = 0x00;
		_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // select register bank 0

		g_i2c_buffer[0] = ICM42688_INTF_CONFIG1;
		g_i2c_buffer[1] = 0x95;
		_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // enable RTC

		g_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
		g_i2c_buffer[1] = 0x01;
		_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // select register bank 1

		g_i2c_buffer[0] = ICM42688_INTF_CONFIG5;
		g_i2c_buffer[1] = 0x04;
		_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // use CLKIN as clock source
	}

	g_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
	g_i2c_buffer[1] = 0x00;
	_i2c_write(ICM42688_ADDRESS, g_i2c_buffer, 2); // select register bank 0

	// For next read
	g_i2c_buffer[0] = ICM42688_TEMP_DATA1;
	g_i2c_buffer[1] = 0x00;
}

static void publish_data(void) {
	publish(SENSOR_IMU_GYRO, (uint8_t*)&g_imu_gyro_accel[3], 6);
	publish(SENSOR_IMU_ACCEL, (uint8_t*)g_imu_gyro_accel, 6);
}

static void calibrate(void) {
	g_imu_calibration[0] += g_imu_gyro_accel[3];
	g_imu_calibration[1] += g_imu_gyro_accel[4];
	g_imu_calibration[2] += g_imu_gyro_accel[5];
	g_calibration_counter++;

	if (g_calibration_counter >= CALIBRATION_FREQ) {
		g_imu_gyro_offset[0] = g_imu_calibration[0] / CALIBRATION_FREQ;
		g_imu_gyro_offset[1] = g_imu_calibration[1] / CALIBRATION_FREQ;
		g_imu_gyro_offset[2] = g_imu_calibration[2] / CALIBRATION_FREQ;
		g_imu_mode = ready;
	}
}

static void imu_init(void) {
	_icm42688p_init(AFS_2G, GFS_1000DPS, AODR_500Hz, GODR_32kHz, aMode_LN, gMode_LN, 0);
}

static void imu_loop(uint8_t *data, size_t size) {
	uint8_t *buffer = &g_i2c_buffer[2];
	_i2c_read(ICM42688_ADDRESS, g_i2c_buffer, 1, buffer, 14);

	// left-, right+, back+, front-
	int16_t ax = (int16_t) (buffer[2] << 8 | buffer[3]);
	int16_t ay = (int16_t) (buffer[4] << 8 | buffer[5]);
	int16_t az = (int16_t) (buffer[6] << 8 | buffer[7]);

	// left-, right+, back-, front+
	int16_t gx = (int16_t) (buffer[8] << 8 | buffer[9]);
	int16_t gy = (int16_t) (buffer[10] << 8 | buffer[11]);
	int16_t gz = (int16_t) (buffer[12] << 8 | buffer[13]);

	g_imu_gyro_accel[0] = ax;
	g_imu_gyro_accel[1] = ay;
	g_imu_gyro_accel[2] = az;
	g_imu_gyro_accel[3] = gx;
	g_imu_gyro_accel[4] = gy;
	g_imu_gyro_accel[5] = gz;

	if (g_imu_mode == ready) {
		g_imu_gyro_accel[3] -= g_imu_gyro_offset[0];
		g_imu_gyro_accel[4] -= g_imu_gyro_offset[1];
		g_imu_gyro_accel[5] -= g_imu_gyro_offset[2];
		publish_data();
	}
	else if (g_imu_mode == calibrating) {
		calibrate();
	}
}

static void imu_calibrate(uint8_t *data, size_t size) {
	g_calibration_counter = 0;
	g_imu_calibration[0] = 0;
	g_imu_calibration[1] = 0;
	g_imu_calibration[2] = 0;
	g_imu_mode = calibrating;
}

void imu_setup(void) {
	imu_init();
	subscribe(SCHEDULER_1KHZ, imu_loop);
	subscribe(COMMAND_CALIBRATE_IMU, imu_calibrate);

	// Try to calibrate once first
	get_platform()->delay(1000);
	publish(COMMAND_CALIBRATE_IMU, NULL, 0);
}
