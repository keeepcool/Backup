/*
* BME280.h
*
* Created: 22/01/2016 11:24:40
*  Author: Tiago
*/


#ifndef BME280_H_
#define BME280_H_

#include "stdint.h"

//Register names:
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB

/******************************************************************************/
/*                          ERROR CODE DEFINITIONS  */
/******************************************************************************/
#define SUCCESS                                        ((char)0)
#define E_BME280_NULL_PTR                                                  ((short)-127)
#define E_BME280_COMM_RES                              ((short)-1)
#define E_BME280_OUT_OF_RANGE                          ((short)-2)
#define ERROR                                          ((char)-1)
/******************************************************************************/
/*                           I2C ADDRESS DEFINITIONS  */
/******************************************************************************/
#define BME280_I2C_ADDRESS1                                         0x76
#define BME280_I2C_ADDRESS2                                         0x77
/******************************************************************************/
/*                            POWER MODE DEFINITIONS  */
/******************************************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE                                           0x00
#define BME280_FORCED_MODE                                          0x01
#define BME280_NORMAL_MODE                                          0x03
#define BME280_SOFT_RESET                                           0xB6
/******************************************************************************/
/*                             STANDBY DEFINITIONS  */
/******************************************************************************/
#define BME280_STANDBY_TIME_1_MS                                    0x00
#define BME280_STANDBY_TIME_63_MS                                   0x01
#define BME280_STANDBY_TIME_125_MS                                  0x02
#define BME280_STANDBY_TIME_250_MS                                  0x03
#define BME280_STANDBY_TIME_500_MS                                  0x04
#define BME280_STANDBY_TIME_1000_MS                                 0x05
#define BME280_STANDBY_TIME_10_MS                                   0x06
#define BME280_STANDBY_TIME_20_MS                                   0x07
/******************************************************************************/
/*                           OVER SAMPLING DEFINITIONS  */
/******************************************************************************/
#define BME280_OVERSAMP_SKIPPED                                     0x00
#define BME280_OVERSAMP_1X                                          0x01
#define BME280_OVERSAMP_2X                                          0x02
#define BME280_OVERSAMP_4X                                          0x03
#define BME280_OVERSAMP_8X                                          0x04
#define BME280_OVERSAMP_16X                                         0x05
/******************************************************************************/
/*                           WORK MODE DEFINITIONS  */
/******************************************************************************/

#define BME280_ULTRALOWPOWER_MODE                                   0x00
#define BME280_LOWPOWER_MODE                                        0x01
#define BME280_STANDARDRESOLUTION_MODE                              0x02
#define BME280_HIGHRESOLUTION_MODE                                  0x03
#define BME280_ULTRAHIGHRESOLUTION_MODE                             0x04

#define BME280_ULTRALOWPOWER_OSRS_P                                 BME280_OVERSAMP_1X
#define BME280_ULTRALOWPOWER_OSRS_T                                 BME280_OVERSAMP_1X

#define BME280_LOWPOWER_OSRS_P                                      BME280_OVERSAMP_2X
#define BME280_LOWPOWER_OSRS_T                                      BME280_OVERSAMP_1X

#define BME280_STANDARDRESOLUTION_OSRS_P                            BME280_OVERSAMP_4X
#define BME280_STANDARDRESOLUTION_OSRS_T                            BME280_OVERSAMP_1X

#define BME280_HIGHRESOLUTION_OSRS_P                                BME280_OVERSAMP_8X
#define BME280_HIGHRESOLUTION_OSRS_T                                BME280_OVERSAMP_1X

#define BME280_ULTRAHIGHRESOLUTION_OSRS_P                           BME280_OVERSAMP_16X
#define BME280_ULTRAHIGHRESOLUTION_OSRS_T                           BME280_OVERSAMP_2X

#define BME280_STANDARD_OVERSAMP_HUMIDITY                           BME280_OVERSAMP_1X
/******************************************************************************/
/*                           FILTER DEFINITIONS  */
/******************************************************************************/
#define BME280_FILTER_COEFF_OFF                                     0x00
#define BME280_FILTER_COEFF_2                                       0x01
#define BME280_FILTER_COEFF_4                                       0x02
#define BME280_FILTER_COEFF_8                                       0x03
#define BME280_FILTER_COEFF_16                                      0x04
/******************************************************************************/
/*                          DELAY DEFINITIONS  */
/******************************************************************************/
#define T_INIT_MAX                                                  20 /* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX                                      37 /* 37/16 = 2.3125 ms*/
#define T_SETUP_PRESSURE_MAX                                        10 /* 10/16 = 0.625 ms */
#define T_SETUP_HUMIDITY_MAX                                        10 /* 10/16 = 0.625 ms */

/******************************************************************************/
/*        DEFINITIONS FOR ARRAY SIZE OF DATA   */
/******************************************************************************/
#define        BME280_HUMIDITY_DATA_SIZE                            2
#define        BME280_TEMPERATURE_DATA_SIZE                         3
#define        BME280_PRESSURE_DATA_SIZE                            3
#define        BME280_DATA_FRAME_SIZE                               8
/* data frames includes temperature, pressure and humidity */
#define        BME280_CALIB_DATA_SIZE                               26

#define        BME280_TEMPERATURE_MSB_DATA                          0
#define        BME280_TEMPERATURE_LSB_DATA                          1
#define        BME280_TEMPERATURE_XLSB_DATA                         2
#define        BME280_PRESSURE_MSB_DATA                             0
#define        BME280_PRESSURE_LSB_DATA                             1
#define        BME280_PRESSURE_XLSB_DATA                            2
#define        BME280_HUMIDITY_MSB_DATA                             0
#define        BME280_HUMIDITY_LSB_DATA                             1

#define        BME280_DATA_FRAME_PRESSURE_MSB_BYTE                  0
#define        BME280_DATA_FRAME_PRESSURE_LSB_BYTE                  1
#define        BME280_DATA_FRAME_PRESSURE_XLSB_BYTE                 2
#define        BME280_DATA_FRAME_TEMPERATURE_MSB_BYTE               3
#define        BME280_DATA_FRAME_TEMPERATURE_LSB_BYTE               4
#define        BME280_DATA_FRAME_TEMPERATURE_XLSB_BYTE              5
#define        BME280_DATA_FRAME_HUMIDITY_MSB_BYTE                  6
#define        BME280_DATA_FRAME_HUMIDITY_LSB_BYTE                  7

/******************************************************************************/
/*                BIT MASK, LENGTH AND POSITION DEFINITIONS  */
/******************************************************************************/
/* Status Register */
#define BME280_STAT_REG_MEASURING__POS                              3
#define BME280_STAT_REG_MEASURING__MSK                              0x08
#define BME280_STAT_REG_MEASURING__LEN                              1
#define BME280_STAT_REG_MEASURING__REG                              BME280_STAT_REG

#define BME280_STAT_REG_IM_UPDATE__POS                              0
#define BME280_STAT_REG_IM_UPDATE__MSK                              0x01
#define BME280_STAT_REG_IM_UPDATE__LEN                              1
#define BME280_STAT_REG_IM_UPDATE__REG                              BME280_STAT_REG
/******************************************************************************/
/*  BIT MASK, LENGTH AND POSITION DEFINITIONS FOR TEMPERATURE OVERSAMPLING  */
/******************************************************************************/
/* Control Measurement Register */
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS              5
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK              0xE0
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN              3
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG              BME280_CTRL_MEAS_REG
/******************************************************************************/
/*     BIT MASK, LENGTH AND POSITION DEFINITIONS FOR PRESSURE OVERSAMPLING  */
/******************************************************************************/
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS                 2
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK                 0x1C
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN                 3
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG                 BME280_CTRL_MEAS_REG
/******************************************************************************/
/*        BIT MASK, LENGTH AND POSITION DEFINITIONS FOR POWER MODE  */
/******************************************************************************/
#define BME280_CTRL_MEAS_REG_POWER_MODE__POS                        0
#define BME280_CTRL_MEAS_REG_POWER_MODE__MSK                        0x03
#define BME280_CTRL_MEAS_REG_POWER_MODE__LEN                        2
#define BME280_CTRL_MEAS_REG_POWER_MODE__REG                        BME280_CTRL_MEAS_REG
/******************************************************************************/
/*    BIT MASK, LENGTH AND POSITION DEFINITIONS FOR HUMIDITY OVERSAMPLING  */
/******************************************************************************/
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__POS             0
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__MSK             0x07
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__LEN             3
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__REG             BME280_CTRL_HUMIDITY_REG
/******************************************************************************/
/*        BIT MASK, LENGTH AND POSITION DEFINITIONS FOR STANDBY TIME  */
/******************************************************************************/
/* Configuration Register */
#define BME280_CONFIG_REG_TSB__POS                                  5
#define BME280_CONFIG_REG_TSB__MSK                                  0xE0
#define BME280_CONFIG_REG_TSB__LEN                                  3
#define BME280_CONFIG_REG_TSB__REG                                  BME280_CONFIG_REG
/******************************************************************************/
/*       BIT MASK, LENGTH AND POSITION DEFINITIONS FOR FILTER  */
/******************************************************************************/
#define BME280_CONFIG_REG_FILTER__POS                               2
#define BME280_CONFIG_REG_FILTER__MSK                               0x1C
#define BME280_CONFIG_REG_FILTER__LEN                               3
#define BME280_CONFIG_REG_FILTER__REG                               BME280_CONFIG_REG
/******************************************************************************/
/*         BIT MASK, LENGTH AND POSITION DEFINITIONS FOR SPI ENABLE  */
/******************************************************************************/
#define BME280_CONFIG_REG_SPI3_ENABLE__POS                          0
#define BME280_CONFIG_REG_SPI3_ENABLE__MSK                          0x01
#define BME280_CONFIG_REG_SPI3_ENABLE__LEN                          1
#define BME280_CONFIG_REG_SPI3_ENABLE__REG                          BME280_CONFIG_REG
/******************************************************************************/
/* BIT MASK, LENGTH AND POSITION DEFINITIONS FOR PRESSURE AND TEMPERATURE DATA  */
/******************************************************************************/
/* Data Register */
#define BME280_PRESSURE_XLSB_REG_DATA__POS                          4
#define BME280_PRESSURE_XLSB_REG_DATA__MSK                          0xF0
#define BME280_PRESSURE_XLSB_REG_DATA__LEN                          4
#define BME280_PRESSURE_XLSB_REG_DATA__REG                          BME280_PRESSURE_XLSB_REG

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS                       4
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK                       0xF0
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN                       4
#define BME280_TEMPERATURE_XLSB_REG_DATA__REG                       BME280_TEMPERATURE_XLSB_REG

#define FIRST_INIT 1

//settings

typedef struct{
	//Main Interface and mode settings

	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	uint8_t humidOverSample;

}SensorSettings_T;

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	uint8_t dig_H6;
}SensorCalibration_T;

//Software init routine
uint8_t BME280_Init(char first);
void BME280_readCalibration(void);

void BME280_debugPrint(void);

void BME280_SoftReset(void);
char BME280_GetStatus(void);
char BME280_GetCtrlMeasurement(void);
char BME280_GetCtrlHumidity(void);
char BME280_GetConfig(void);

void BME280_SetOversamplingPressure(char Value);
void BME280_SetOversamplingTemperature(char Value);
void BME280_SetOversamplingHumidity(char Value);
void BME280_SetOversamplingMode(char Value);
void BME280_SetFilterCoefficient(char Value);
void BME280_SetStandbyTime(char Value);

//Returns the values as floats.
float BME280_readFloatPressure(void);
float BME280_readFloatAltitudeMeters(void);
float BME280_readFloatHumidity( void );
float BME280_readTempC( void );
char BME280_IsMeasuring();

//The following utilities read and write
//readRegister reads one register
uint8_t BME280_readRegister(uint8_t);
//Writes a byte;
void BME280_writeRegister(uint8_t, uint8_t);
char BME280_GetID(void);


#endif /* BME280_H_ */