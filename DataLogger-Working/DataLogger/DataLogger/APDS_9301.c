/*
 * APDS_9301.c
 *
 * Created: 26/01/2016 15:45:12
 *  Author: Tiago
 */ 

#define F_CPU 16000000UL

//#define ADPS_DEBUG

#ifdef ADPS_DEBUG
	char buff[64];
#endif // ADSP_DEBUG

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
#include "APDS_9301.h"



void APDS_Init(void){
	
	#ifdef ADPS_DEBUG
	uartPutsP("Inside Init\n");
	#endif // ADSP_DEBUG
	
	APDS_setIntegrationTime(INTEG_SLOW);
	APDS_setGain(0);
	APDS_setPower(POWER_UP);
	
}

void APDS_setPower(uint8_t powerSetting){
	
	#ifdef ADPS_DEBUG
	uartPutsP("Inside Power ON\n");
	#endif // ADSP_DEBUG
	
	APDS_writeRegister(CONTROL, powerSetting);
}

float APDS_getLux(void){
	
	#ifdef ADPS_DEBUG
	uartPutsP("Inside getLux\n");
	#endif // ADSP_DEBUG
	
	float result = 0.0f;
	uint16_t CH0 = APDS_readRegister(DATA0LOW);
	uint16_t CH1 = APDS_readRegister(DATA1LOW);
	
	float channelRatio = (CH1*1.0f)/(CH0*1.0f);//float(CH1)/float(CH0);
	#ifdef ADPS_DEBUG
	sprintf(buff, "CH0: %u\nCH1: %u\nRatio: %f\n", CH0, CH1, channelRatio);
	uartPuts(buff);
	#endif
	
	if(channelRatio <= 0.5f){
		result = (0.0224f * CH0) - (0.031f * CH1);}
	else if(channelRatio <= 0.61f){
		result = (0.0128f * CH0) - (0.0153f * CH1);}
	else if(channelRatio <= 0.80f){
		result = (0.0128f * CH0) - (0.0153f * CH1); }
	else if(channelRatio <= 1.30f){
		result = (0.00146f * CH0) - (0.00112f * CH1); }
	
	if(channelRatio >1.3f){
		result = 0.0;
	}
	
	return result;
	
}

void APDS_setGain(uint8_t setGain){
	
	APDS_writeRegister(TIMING, setGain);	
	
}

void APDS_setIntegrationTime(uint8_t setInteg){
	
	APDS_writeRegister(TIMING, setInteg);
	
}

uint8_t APDS_readID(void){
	
	uint8_t temp = 0;
	
	#ifdef ADPS_DEBUG
	uartPutsP("Inside ReadID\n");
	#endif // ADSP_DEBUG
	
	temp = APDS_readRegister(APDS_ID);
	
	return temp;
	
}


uint16_t APDS_readRegister(uint8_t offset){
	
	uint16_t result=0xFFFF;
	uint8_t word=0;
	uint16_t c = 0;
	uint8_t temp = 0;
	
	#ifdef ADPS_DEBUG
	uartPutsP("Inside ReadReg\n");
	#endif // ADSP_DEBUG
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	switch(offset){
		case CONTROL:
		case TIMING:
		case INTERRUPT:
		case APDS_ID:
		word = 0;
		break;

		case THRESHLOWLOW:
		case THRESHHIGHLOW:
		case DATA0LOW:
		case DATA1LOW:
		word = 0x20;
		break;
	}
	
	temp = (0x80 | word | offset);
	i2c_start(APDS_ADDRESS+I2C_WRITE);
	
	#ifdef ADPS_DEBUG
	sprintf(buff, "i2c Start:%s, %i\n", error ? "NOT FOUND":"OK", error);
	uartPuts(buff);
	sprintf(buff, "Writing: %x\n", temp);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	i2c_write(temp);
	
	
	//c = i2c_readAck();
	if(word){
		
		#ifdef ADPS_DEBUG
		uartPutsP("Its a WORD!\n");
		#endif // ADSP_DEBUG
		i2c_start(APDS_ADDRESS+I2C_READ);
		c = i2c_readAck();
		result = (c<<8) | i2c_readNak();
		
		} else {
		
		#ifdef ADPS_DEBUG
		uartPutsP("Its a BYTE!\n");
		#endif // ADSP_DEBUG
		i2c_rep_start(APDS_ADDRESS+I2C_READ);
		result = i2c_readNak();
		
	}
	
	i2c_stop();
	return result;
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
}


void APDS_writeRegister(uint8_t offset, uint16_t dataToWrite){
	
	uint8_t word=0;
	uint8_t temp = 0;
	
	#ifdef ADPS_DEBUG
	uartPutsP("\n\nInside writeReg!\n");
	#endif // ADSP_DEBUG
	
	CTRL_PORT &= ~_BV(PLEN_PIN);	//Make sure pull ups are on
	
	switch(offset){
		case CONTROL:
		case TIMING:
		case INTERRUPT:
		case APDS_ID:
		word = 0;
		break;

		case THRESHLOWLOW:
		case THRESHHIGHLOW:
		case DATA0LOW:
		case DATA1LOW:
		word = 0x20;
		break;
	}
	
	#ifdef ADPS_DEBUG
	//char buff[32];
	sprintf(buff, "Switch result: %x\n", word);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	i2c_start(APDS_ADDRESS+I2C_WRITE);
	
	#ifdef ADPS_DEBUG
	sprintf(buff, "i2c Start:%s, %i\n", error ? "NOT FOUND":"OK", error);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	temp =(0x80 | word | offset);
	
	#ifdef ADPS_DEBUG
	sprintf(buff, "i2c write: %x\n", temp);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	i2c_write(temp);
	
	#ifdef ADPS_DEBUG
	sprintf(buff, "Write done\nE:%i\n", error);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	temp = (uint8_t)(dataToWrite & 0x00FF);
	
	//i2c_rep_start(ADSP_ADDRESS+I2C_WRITE);
	#ifdef ADPS_DEBUG
	sprintf(buff, "i2c write:%i\n", temp);
	uartPuts(buff);
	#endif // ADSP_DEBUG
	
	i2c_write(temp);
	
	temp = (dataToWrite>>8 & 0xFF);
	
	if(word){
		
		#ifdef ADPS_DEBUG
		uartPutsP("Its a WORD!\n");
		#endif // ADSP_DEBUG
		
		i2c_write(temp);

		}
	i2c_stop();
	
	CTRL_PORT |= _BV(PLEN_PIN);	//Make sure pull ups are off
}