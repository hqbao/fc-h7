#include "dps310_platform.h"
#include "dps310_errors.h"
#include <platform.h>

char _i2c_read(uint16_t address, uint8_t *data, uint16_t size) {
	return paltform_i2c_read(I2C_PORT3, address, data, size);
}

char _i2c_write(uint16_t address, uint8_t *data, uint16_t size) {
	return paltform_i2c_write(I2C_PORT3, address, data, size);
}

void dps310_i2c_delay_ms(uint32_t delay) {
	get_platform()->delay(delay);
}

void dps310_i2c_init(void) {}

void dps310_i2c_release(void) {}

void dps310_enable(void) {}

char dps310_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count) {
	char ret= DPS310_I2C_FAIL_ERROR;
    uint8_t buff[1];

    buff[0] = reg;

    ret = _i2c_write((uint16_t)address, buff, 1);
    if (ret != 0) return DPS310_I2C_FAIL_ERROR;

    dps310_i2c_delay_ms(1); //10ms

    ret = _i2c_read((uint16_t)address, data, count);
    if (ret != 0) return DPS310_I2C_FAIL_ERROR;

    return DPS310_OK;
}

char dps310_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count) {
	char ret = DPS310_I2C_FAIL_ERROR;
	uint16_t count_with_reg = count + 1;
	uint8_t buff[DPS310_I2C_MAX_BUFF_SIZE];

	buff[0] = reg;

	for (uint8_t i = 1; i < count_with_reg; i++) {
		buff[i] = data[i - 1];
	}

	ret = _i2c_write((uint16_t)address, buff, count_with_reg);
	if (ret != 0) return DPS310_I2C_FAIL_ERROR;

	return DPS310_OK;
}
