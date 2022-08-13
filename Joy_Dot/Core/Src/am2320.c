#include "am2320.h"

Am2320_HandleTypeDef am2320_Init(I2C_HandleTypeDef* i2c_handle,uint8_t device_address) {
	Am2320_HandleTypeDef AM2320_;
	AM2320_.i2c_handle = i2c_handle;
	AM2320_.device_address = device_address;
	return AM2320_;
}

uint8_t am2320_ReadValue(Am2320_HandleTypeDef *am2320) {
	uint8_t registers[3] = { 0x03, 0x00, 0x04 };
	HAL_I2C_Master_Transmit(am2320->i2c_handle, am2320->device_address, 0x00, 0,
	HAL_MAX_DELAY);
	HAL_Delay(1);
	if (HAL_I2C_Master_Transmit(am2320->i2c_handle, am2320->device_address,registers, 3, HAL_MAX_DELAY) != HAL_OK) {
		return 1;
	}
	HAL_Delay(2);
	if (HAL_I2C_Master_Receive(am2320->i2c_handle, am2320->device_address,am2320->data, 8, HAL_MAX_DELAY) != HAL_OK) {
		return 2;
	}
	if (am2320->data[1] != 0x04 && am2320->data[0] != 0x03) {
		return 3;
	}
	return 0;
}

void am2320_GetTemperatureAndHumidity(Am2320_HandleTypeDef *am2320, float *temperature,float *humidity) {
	int read = am2320_ReadValue(am2320);
	if (read) {
		// Simple exception handling
	}
	uint16_t temp_temperature = (am2320->data[5] | am2320->data[4] << 8);
	if (temp_temperature & 0x8000) {
		temp_temperature = -(int16_t) (temp_temperature & 0x7fff);
	} else {
		temp_temperature = (int16_t) temp_temperature;
	}
	*temperature = (float) temp_temperature / 10.0;
	*humidity = (float) (am2320->data[3] | am2320->data[2] << 8) / 10.0;
}
