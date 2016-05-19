/*
 * si70xx.c
 *
 * Created: 13/05/2016 10:18:48
 *  Author: Tiago
 */ 

#include "si70xx.h"


void si70xx_init(void) {
	i2c_start(SHT2x_I2CADDR_WRITE);
	i2c_write(SHT2x_CMD_SOFT_RESET);
	_delay_ms(15); // As per datasheet
	i2c_stop();
}

// Read temperature
float si70xx_get_temperature(void) {
	i2c_start(SHT2x_I2CADDR_WRITE);
	i2c_write(SHT2x_CMD_TEMPERATURE_HOLD);
	_delay_ms(15); // Wait for conversion
	i2c_stop();
	i2c_start(SHT2x_I2CADDR_READ);
	uint16_t result = (i2c_readAck() << 8);
	result += i2c_readNak();
	result &= ~0x03; // Clear status bits
	i2c_stop();
	return (-46.85 + 175.72 / 65536.0 * (float)(result)); // As per datasheet
}

// Read humidity
float si70xx_get_humidity(void) {
	i2c_start(SHT2x_I2CADDR_WRITE);
	i2c_write(SHT2x_CMD_HUMIDITY_HOLD);
	_delay_ms(15); // Wait for conversion
	i2c_stop();
	i2c_start(SHT2x_I2CADDR_READ);
	uint16_t result = (i2c_readAck() << 8);
	result += i2c_readNak();
	result &= ~0x03; // Clear status bits
	i2c_stop();
	return (-6.0 + 125.0 / 65536.0 * (float)(result)); // As per datasheet
}