#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include <stdlib.h>
#include "icm42688p.h"

#define CALIBRATION_FREQ 16000 // 2 seconds
#define IMU_MOTION 200
#define SSF_GYRO (16.4)

typedef enum {
	init = 0,
	calibrating,
	ready,
} imu_mode_t;

static uint8_t g_imu_i2c_buffer[32] = {0};
static float g_imu_gyro_accel[6] = {0};
static int64_t g_imu_calibration[3] = {0};
static int16_t g_imu_calibration_check[3] = {0};
static float g_imu_gyro_offset[3] = {0};
static int g_imu_calibration_counter = 0;
static imu_mode_t g_imu_mode = init;

static void _i2c_write_read(uint8_t address, uint8_t *input, uint16_t input_size,
		uint8_t* output, uint16_t output_size, uint32_t timeout) {
	platform_i2c_write_read(I2C_PORT1, address, input, input_size, output, output_size, timeout);
}

static void _i2c_write_read_dma(uint8_t address, uint8_t *input, uint16_t input_size,
		uint8_t* output, uint16_t output_size) {
	platform_i2c_write_read_dma(I2C_PORT1, address, input, input_size, output, output_size);
}

static void _i2c_write(uint8_t address, uint8_t *data, uint16_t size) {
	platform_i2c_write(I2C_PORT1, address, data, size);
}

static void _icm42688p_init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR,
                           uint8_t aMode, uint8_t gMode, bool CLKIN) {
    // Ensure register bank 0 is selected
    g_imu_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    g_imu_i2c_buffer[1] = 0x00;
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Verify WHO_AM_I (optional but recommended)
    g_imu_i2c_buffer[0] = ICM42688_WHO_AM_I;
    _i2c_write_read(ICM42688_ADDRESS, g_imu_i2c_buffer, 1, &g_imu_i2c_buffer[1], 1, 1000);

    // Set power modes (gyro in low-noise mode, accel off if not needed)
    g_imu_i2c_buffer[0] = ICM42688_PWR_MGMT0;
    g_imu_i2c_buffer[1] = (gMode << 2) | aMode;  // gMode = 0x03 (LN mode), aMode = 0x00 (off)
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);
    platform_delay(1);

    // Configure accelerometer (if used, else skip)
    if (aMode != 0x00) {
        g_imu_i2c_buffer[0] = ICM42688_ACCEL_CONFIG0;
        g_imu_i2c_buffer[1] = (Ascale << 5) | AODR;
        _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);
    }

    // Configure gyro: 8 kHz ODR + desired scale (e.g., 2000dps)
    g_imu_i2c_buffer[0] = ICM42688_GYRO_CONFIG0;
    g_imu_i2c_buffer[1] = (Gscale << 5) | GODR;  // GODR = 0x08 (8 kHz)
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Set gyro bandwidth to ODR/2 (4 kHz) for minimal filtering
    g_imu_i2c_buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
    g_imu_i2c_buffer[1] = 0x07;  // Gyro BW = 000 (ODR/2), Accel BW = 111 (ODR/320)
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Enable FIFO for gyro data (critical for 8 kHz streaming)
    g_imu_i2c_buffer[0] = ICM42688_FIFO_CONFIG1;
    g_imu_i2c_buffer[1] = 0x03;  // FIFO mode + gyro data stored
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    g_imu_i2c_buffer[0] = ICM42688_TMST_CONFIG;
    g_imu_i2c_buffer[1] = 0x01; // Enable temp compensation
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Configure interrupts (data-ready on INT1)
    g_imu_i2c_buffer[0] = ICM42688_INT_CONFIG;
    g_imu_i2c_buffer[1] = 0x18 | 0x03;  // Push-pull, pulsed, active HIGH
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    g_imu_i2c_buffer[0] = ICM42688_INT_SOURCE0;
    g_imu_i2c_buffer[1] = 0x08;  // Data-ready interrupt to INT1
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Optional: External clock (if CLKIN=true)
    if (CLKIN) {
        g_imu_i2c_buffer[0] = ICM42688_INTF_CONFIG1;
        g_imu_i2c_buffer[1] = 0x95;  // Enable RTC
        _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

        g_imu_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
        g_imu_i2c_buffer[1] = 0x01;  // Bank 1
        _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

        g_imu_i2c_buffer[0] = ICM42688_INTF_CONFIG5;
        g_imu_i2c_buffer[1] = 0x04;  // Use CLKIN
        _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

        // Return to bank 0
        g_imu_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
        g_imu_i2c_buffer[1] = 0x00;
        _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);
    }

    // Bank 2
    g_imu_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    g_imu_i2c_buffer[1] = 0x02;
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    g_imu_i2c_buffer[0] = ICM42688_GYRO_CONFIG_STATIC2;
    g_imu_i2c_buffer[1] = 0x00; // Disable AA filter
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Return to Bank 0
    g_imu_i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    g_imu_i2c_buffer[1] = 0x00;
    _i2c_write(ICM42688_ADDRESS, g_imu_i2c_buffer, 2);

    // Prepare for reading (temperature register as placeholder)
    g_imu_i2c_buffer[0] = ICM42688_TEMP_DATA1;
}

static void publish_data(void) {
	g_imu_gyro_accel[3] = g_imu_gyro_accel[3] / SSF_GYRO;
	g_imu_gyro_accel[4] = g_imu_gyro_accel[4] / SSF_GYRO;
	g_imu_gyro_accel[5] = g_imu_gyro_accel[5] / SSF_GYRO;
	publish(SENSOR_IMU_GYRO_UPDATE, (uint8_t*)&g_imu_gyro_accel[3], 12);
	publish(SENSOR_IMU_ACCEL_UPDATE, (uint8_t*)g_imu_gyro_accel, 12);
}

static void calibrate(void) {
	g_imu_calibration[0] += g_imu_gyro_accel[3];
	g_imu_calibration[1] += g_imu_gyro_accel[4];
	g_imu_calibration[2] += g_imu_gyro_accel[5];
	g_imu_calibration_counter++;

	// Check motionless
	char failed = 0;
	if (abs(g_imu_gyro_accel[3] - g_imu_calibration_check[0]) > IMU_MOTION) failed = 1;
	if (abs(g_imu_gyro_accel[4] - g_imu_calibration_check[1]) > IMU_MOTION) failed = 1;
	if (abs(g_imu_gyro_accel[5] - g_imu_calibration_check[2]) > IMU_MOTION) failed = 1;
	g_imu_calibration_check[0] = g_imu_gyro_accel[3];
	g_imu_calibration_check[1] = g_imu_gyro_accel[4];
	g_imu_calibration_check[2] = g_imu_gyro_accel[5];
	if (failed == 1) {
		g_imu_mode = init;
		uint8_t result = 0;
		publish(SENSOR_IMU_GYRO_CALIBRATION_UPDATE, (uint8_t*)&result, 1);
		return;
	}

	if (g_imu_calibration_counter >= CALIBRATION_FREQ) {
		g_imu_gyro_offset[0] = (double)(1.0 / CALIBRATION_FREQ) * g_imu_calibration[0];
		g_imu_gyro_offset[1] = (double)(1.0 / CALIBRATION_FREQ) * g_imu_calibration[1];
		g_imu_gyro_offset[2] = (double)(1.0 / CALIBRATION_FREQ) * g_imu_calibration[2];
		g_imu_mode = ready;
		uint8_t result = 1;
		publish(SENSOR_IMU_GYRO_CALIBRATION_UPDATE, (uint8_t*)&result, 1);
	}
}

static void imu_init(void) {
	_icm42688p_init(AFS_2G, GFS_2000DPS, GODR_25Hz, GODR_8kHz, aMode_LN, gMode_LN, 0);
}

static void imu_loop(uint8_t *data, size_t size) {
	uint8_t *buffer = &g_imu_i2c_buffer[2];
	_i2c_write_read_dma(ICM42688_ADDRESS, g_imu_i2c_buffer, 1, buffer, 14);

	int16_t ax = (int16_t) (buffer[2] << 8 | buffer[3]);
	int16_t ay = (int16_t) (buffer[4] << 8 | buffer[5]);
	int16_t az = (int16_t) (buffer[6] << 8 | buffer[7]);
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
	g_imu_calibration_counter = 0;
	g_imu_calibration[0] = 0;
	g_imu_calibration[1] = 0;
	g_imu_calibration[2] = 0;
	g_imu_calibration_check[0] = g_imu_gyro_accel[3];
	g_imu_calibration_check[1] = g_imu_gyro_accel[4];
	g_imu_calibration_check[2] = g_imu_gyro_accel[5];
	g_imu_mode = calibrating;
}

void imu_setup(void) {
	imu_init();
	subscribe(SCHEDULER_8KHZ, imu_loop);
	subscribe(SENSOR_IMU_CALIBRATE_GYRO, imu_calibrate);
}
