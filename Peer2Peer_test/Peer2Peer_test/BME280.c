/*
 * BME280.c
 *
 * Created: 22/01/2016 11:20:08
 *  Author: Tiago
 */ 

#define F_CPU		16000000UL
#define I2C_ADDRESS	(0xEC) //SDO to GND so last bit from address is 0
//#define BME_DEBUG

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "pinMap.h"
#include "i2cmaster.h"
#include "uart.h"
#include "BME280.h"

//Global variables 
 SensorSettings_T settings;
 SensorCalibration_T calibration;
 int32_t t_fine;	//Fine temperature measure used to compensate the humidity and pressure values

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280_Init(char first){
	//Check the settings structure values to determine how to setup the device
	
	//Construct with these default settings if nothing is specified
	settings.runMode = 0;
	settings.tempOverSample = 0;
	settings.pressOverSample = 0;
	settings.humidOverSample = 0;
	
	//	uint8_t dataToWrite = 0;  //Temporary variable
	
	#ifdef BME_DEBUG
	uartPutsP("\nStarting BME debug\n");
	uartPutsP("Pull-ups are enabled\n");
	#endif // BME_DEBUG
	
	//i2c_init();	//Make sure i2c is up and running
	
	CTRL_PORT &= ~_BV(PLEN_PIN);
	
	
	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	if(first == FIRST_INIT){
		BME280_readCalibration();
	}
	
	BME280_SetStandbyTime(BME280_STANDBY_TIME_1_MS);                              // Standby time 1ms
	BME280_SetFilterCoefficient(BME280_FILTER_COEFF_16);                          // IIR Filter coefficient 16
	BME280_SetOversamplingPressure(BME280_OVERSAMP_8X);                          // Pressure x16 oversampling
	BME280_SetOversamplingTemperature(BME280_OVERSAMP_2X);                        // Temperature x2 oversampling
	BME280_SetOversamplingHumidity(BME280_OVERSAMP_1X);                           // Humidity x1 oversampling
	BME280_SetOversamplingMode(BME280_NORMAL_MODE);
	return BME280_readRegister(BME280_CHIP_ID_REG);
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
}

void BME280_readCalibration(void){
	
	calibration.dig_T1 = ((uint16_t)((BME280_readRegister(BME280_DIG_T1_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((BME280_readRegister(BME280_DIG_T2_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((BME280_readRegister(BME280_DIG_T3_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((BME280_readRegister(BME280_DIG_P1_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((BME280_readRegister(BME280_DIG_P2_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((BME280_readRegister(BME280_DIG_P3_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((BME280_readRegister(BME280_DIG_P4_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((BME280_readRegister(BME280_DIG_P5_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((BME280_readRegister(BME280_DIG_P6_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((BME280_readRegister(BME280_DIG_P7_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((BME280_readRegister(BME280_DIG_P8_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((BME280_readRegister(BME280_DIG_P9_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(BME280_readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((BME280_readRegister(BME280_DIG_H2_MSB_REG) << 8) + BME280_readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(BME280_readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((BME280_readRegister(BME280_DIG_H4_MSB_REG) << 4) + (BME280_readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((BME280_readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((BME280_readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((uint8_t)BME280_readRegister(BME280_DIG_H6_REG));
}

void BME280_debugPrint(void){
	
	CTRL_PORT &= ~_BV(PLEN_PIN);
	char Lbuff[64];
	int32_t adc_T = ((uint32_t)BME280_readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)BME280_readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((BME280_readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

	sprintf(Lbuff, "chip ID: %i\n", BME280_GetID());
	uartPuts(Lbuff);
	sprintf(Lbuff, "Raw temp: %li \n", adc_T);
	uartPuts(Lbuff);
	sprintf(Lbuff, "Temperature: %f \n", BME280_readTempC());
	uartPuts(Lbuff);
	sprintf(Lbuff, "Pressure: %f \n", BME280_readFloatPressure());
	uartPuts(Lbuff);
	sprintf(Lbuff, "Altitude: %f \n", BME280_readFloatAltitudeMeters());
	uartPuts(Lbuff);
	sprintf(Lbuff, "Humidity: %f \n", BME280_readFloatHumidity());
	uartPuts(Lbuff);
	sprintf(Lbuff, "Status(%x): %i \n", BME280_STAT_REG , BME280_GetStatus());
	uartPuts(Lbuff);
	sprintf(Lbuff, "CTRL_MEAS(%x): %i \n", BME280_CTRL_MEAS_REG, BME280_GetCtrlMeasurement());
	uartPuts(Lbuff);
	sprintf(Lbuff, "CTRL_HUM(%x): %i \n", BME280_CTRL_HUMIDITY_REG, BME280_GetCtrlHumidity());
	uartPuts(Lbuff);
	sprintf(Lbuff, "CONFIG(%x): %i \n", BME280_CONFIG_REG, BME280_GetConfig());
	uartPuts(Lbuff);
	
}


char BME280_GetID() {
	return BME280_readRegister(BME280_CHIP_ID_REG);
}

void BME280_SoftReset() {
	BME280_writeRegister(BME280_RST_REG, BME280_SOFT_RESET);
}

char BME280_GetStatus() {
	return BME280_readRegister(BME280_STAT_REG);
}

char BME280_GetCtrlMeasurement() {
	return BME280_readRegister(BME280_CTRL_MEAS_REG);
}

char BME280_GetCtrlHumidity() {
	return BME280_readRegister(BME280_CTRL_HUMIDITY_REG);
}

char BME280_GetConfig() {
	return BME280_readRegister(BME280_CONFIG_REG);
}


void BME280_SetOversamplingPressure(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK;
	ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS;
	
	BME280_writeRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingTemperature(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK;
	ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS;

	BME280_writeRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingHumidity(char Value) {

	BME280_writeRegister(BME280_CTRL_HUMIDITY_REG, Value );
}

void BME280_SetOversamplingMode(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm |= Value;

	BME280_writeRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetFilterCoefficient(char Value) {
	char cfgv;
	cfgv = BME280_GetConfig();
	cfgv &= ~BME280_CONFIG_REG_FILTER__MSK;
	cfgv |= Value << BME280_CONFIG_REG_FILTER__POS;
}

void BME280_SetStandbyTime(char Value) {
	char cfgv;
	cfgv = BME280_GetConfig();
	cfgv &= ~BME280_CONFIG_REG_TSB__MSK;
	cfgv |= Value << BME280_CONFIG_REG_TSB__POS;
}

char BME280_IsMeasuring() {
	char output;
	output = BME280_GetStatus();
	return (output & BME280_STAT_REG_MEASURING__MSK);
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float BME280_readFloatPressure(void){

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	int32_t adc_P = ((uint32_t)BME280_readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)BME280_readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((BME280_readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000UL;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17UL);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35UL);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	p_acc = p_acc >> 8; // /256
	return (float)p_acc;
	
}

float BME280_readFloatAltitudeMeters(void){
	float heightOutput = 0;
	
	heightOutput = ((float)-45846.2)*(pow(((float)BME280_readFloatPressure()/(float)101325), 0.190263) - (float)1.0f);
	return heightOutput;
	
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280_readFloatHumidity(void){
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
	int32_t adc_H = ((uint32_t)BME280_readRegister(BME280_HUMIDITY_MSB_REG) << 8) | ((uint32_t)BME280_readRegister(BME280_HUMIDITY_LSB_REG));
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)((var1>>12) >> 10);

}



//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

float BME280_readTempC( void )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
	int32_t adc_T = ((uint32_t)BME280_readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)BME280_readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((BME280_readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100;
	
	return output;
}


uint8_t BME280_readRegister(uint8_t offset){
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	//Return value
	uint8_t result;
	
	i2c_start(I2C_ADDRESS+I2C_WRITE);
	i2c_write(offset);
	i2c_rep_start(I2C_ADDRESS+I2C_READ);
	result = i2c_readNak();
	i2c_stop();
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
	return result;
}


void BME280_writeRegister(uint8_t offset, uint8_t dataToWrite){
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	i2c_start(I2C_ADDRESS+I2C_WRITE);
	i2c_write(offset);
	i2c_write(dataToWrite);
	i2c_stop();
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
}