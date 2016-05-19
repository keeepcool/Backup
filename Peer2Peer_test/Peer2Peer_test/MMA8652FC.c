/*
 * MMA8652FC.c
 *
 * Created: 01/02/2016 18:34:48
 *  Author: Tiago
 */ 


#define F_CPU	16000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>
#include "pinMap.h"
#include "i2cmaster.h"
#include "uart.h"
#include "MMA8652FC.h"

//#define MMA_DEBUG

#ifdef MMA_DEBUG
	char tempBuff[64];
#endif

void MMA8652_Init(uint8_t range){
	
	//Tentative initial settings
	//Call it testing settings not totally aimed at low power
	#ifdef MMA_DEBUG
	uartPutsP("\nMMA8652 Init\n");
	#endif
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	//Start with a soft reset
	//MMA8652_writeRegister(CTRL_REG2, _BV(RST));
	MMA8652_writeRegister(CTRL_REG1, 0x00);	//Make sure its all zeros
	_delay_ms(10);
	
	switch(range){
					case RANGE_2G:
					
					#ifdef MMA_DEBUG
					uartPutsP("Case 2G\n");
					#endif
					MMA8652_writeRegister(XYZ_DATA_CFG, _BV(RANGE_2G));
					break;
					
					case RANGE_4G:
					#ifdef MMA_DEBUG
					uartPutsP("Case 4G\n");
					#endif
					MMA8652_writeRegister(XYZ_DATA_CFG, _BV(RANGE_4G));
					break;
					
					case RANGE_8G:
					#ifdef MMA_DEBUG
					uartPutsP("Case 8G\n");
					#endif
					MMA8652_writeRegister(XYZ_DATA_CFG, _BV(RANGE_8G));
					break;
					
					default:
					#ifdef MMA_DEBUG
					uartPutsP("Case Default\n");
					#endif
					MMA8652_writeRegister(XYZ_DATA_CFG, (RANGE_2G));
					break;
	}
	
	MMA8652_writeRegister(CTRL_REG1, _BV(ACTIVE));
	
	#ifdef MMA_DEBUG
	uartPutsP("MMA8652 Init Done\n");
	#endif
	
}

uint8_t MMA8652_getID(void){
	
	return MMA8652_readRegister(WHO_AM_I);
}

void MMA8652_printID(void){
	
	char localBuff[32];
	uint8_t response = MMA8652_getID();
	sprintf(localBuff,"\nGot: %i Expected: %i %s\n",  response, EXPECTED_ID, ((response == EXPECTED_ID) ? "OK" : "ERROR"));
	uartPuts(localBuff);
	
}

void MMA8652_readAcc(int16_t *dataOut){
	
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	int16_t dataArray[7];
	
	i2c_start(MMA_ADDRESS+I2C_WRITE);
	i2c_write(0x00);
	i2c_start(MMA_ADDRESS+I2C_READ);
	dataArray[0] = i2c_readAck();
	dataArray[1] = i2c_readAck();
	dataArray[2] = i2c_readAck();
	dataArray[3] = i2c_readAck();
	dataArray[4] = i2c_readAck();
	dataArray[5] = i2c_readAck();
	dataArray[6] = i2c_readNak();
	i2c_stop();
	
	//q16 = (q12 & 0x0800) ? (q12 | 0xf800) : q12;
	
	
	dataOut[0] = ((dataArray[1]<<8)|dataArray[2])>>4;
	dataOut[1] = ((dataArray[3]<<8)|dataArray[4])>>4;
	dataOut[2] = ((dataArray[5]<<8)|dataArray[6])>>4;
	
	dataOut[0] = (dataOut[0] & 0x0800) ? (dataOut[0] | 0xf800) : dataOut[0];
	dataOut[1] = (dataOut[1] & 0x0800) ? (dataOut[1] | 0xf800) : dataOut[1];
	dataOut[2] = (dataOut[2] & 0x0800) ? (dataOut[2] | 0xf800) : dataOut[2];
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
}

void MMA8652_readAccG(float *gAxis){
	
	#ifdef MMA_DEBUG
	uartPuts("MMA8652 Read G's\n");
	#endif
	
	int16_t rawAxis[4];
	uint8_t actualRange = 0;
	
	MMA8652_readAcc(rawAxis);
	actualRange = MMA8652_readRegister(XYZ_DATA_CFG) & (0x02);
	
	switch(actualRange){
		case RANGE_2G:
		gAxis[0] = (float)rawAxis[0] / 512.0f;
		gAxis[1] = (float)rawAxis[1] / 512.0f;
		gAxis[2] = (float)rawAxis[2] / 512.0f;
		break;
		
		case RANGE_4G:
		gAxis[0] = (float)rawAxis[0] / 256.0f;
		gAxis[1] = (float)rawAxis[1] / 256.0f;
		gAxis[2] = (float)rawAxis[2] / 256.0f;
		break;
		
		case RANGE_8G:
		gAxis[0] = (float)rawAxis[0] / 128.0f;
		gAxis[1] = (float)rawAxis[1] / 128.0f;
		gAxis[2] = (float)rawAxis[2] / 128.0f;
		break;
		
	}
	
	#ifdef MMA_DEBUG
	sprintf(tempBuff, "Range: %i\n", actualRange);
	uartPuts(tempBuff);
	#endif
		
}

uint8_t MMA8652_readRegister(uint8_t offset){
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	uint8_t result = 0;
	
	i2c_start(MMA_ADDRESS+I2C_WRITE);
	i2c_write(offset);
	i2c_start(MMA_ADDRESS+I2C_READ);
	result = i2c_readNak();
	i2c_stop();
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
	return result;	
}

void MMA8652_writeRegister(uint8_t offset, uint8_t dataToWrite){
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	i2c_start(MMA_ADDRESS+I2C_WRITE);
	i2c_write(offset);
	i2c_write(dataToWrite);
	i2c_stop();
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
	
}