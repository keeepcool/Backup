/*
 * APDS_9301.h
 *
 * Created: 26/01/2016 15:45:18
 *  Author: Tiago
 */ 


#ifndef APDS_9301_H_
#define APDS_9301_H_

#include <stdint.h>

#define APDS_ADDRESS (0x29<<1)

#define CONTROL	0x00		//Control of basic functions
#define TIMING	0x01		//Integration time/gain control
#define THRESHLOWLOW 0x02	//Low byte of low interrupt threshold
#define THRESHLOWHIGH 0x03  //High byte of low interrupt threshold
#define THRESHHIGHLOW 0x04	//Low byte of high interrupt threshold
#define THRESHHIGHHIGH 0x05	//High byte of high interrupt threshold
#define INTERRUPT 0x06		//Interrupt control
#define RES1 0x07			// Reserved
#define CRC 0x08			//Factory test — not a user register
#define RES2 0x09			//Reserved
#define APDS_ID 0x0A		//ID Part number/ Rev ID
#define RES3 0x0B			//Reserved
#define DATA0LOW 0x0C		//Low byte of ADC channel 0
#define DATA0HIGH 0x0D		//High byte of ADC channel 0
#define DATA1LOW 0x0E		//Low byte of ADC channel 1
#define DATA1HIGH 0x0F		//High byte of ADC channel

#define CMD		0x80		//Select command register, set to 1
#define CLEAR	0x40		//Clears interrupt bitwise with inself to clear
#define WORD	0x20		//Set to 1 to read a word via i2c

#define POWER_UP	0b00000011
#define POWER_DOWN	0x00

#define GAIN		0x10	//0 is low gain(1x) 1 is high gain(16x)
#define MANUAL		0x08	//Set to 1 to begin a manual integration, INTEG must be set to 0b11 to work
#define INTEG		0x03	//Integration time, options bellow
#define INTEG_FAST	0x00	//Fast integration time 13.7ms
#define INTEG_MED	0x01	//Medium integration time 101ms
#define INTEG_SLOW	0x02	//Slow integration time 402ms
#define INTEG_MAN	0x03	//Set to this for manual integration times


#define INTR		0x03
#define INTR_OFF	0x00
#define INTR_ON		0x01

#define PERSIST		0x0F	//Goes from 0 to 0x0F ranging from 1 int for each adc clock up to 15 integration times

#define PARTNO		0xF0
#define REVNO		0x0F
#define ID_EXP		0x50

void APDS_Init(void);
void APDS_setPower(uint8_t powerSetting);
uint8_t APDS_readID(void);
uint16_t APDS_readRegister(uint8_t offset);
void APDS_writeRegister(uint8_t offset, uint16_t dataToWrite);
float APDS_getLux(void);
void APDS_setGain(uint8_t setGain);
void APDS_setIntegrationTime(uint8_t setInteg);

#endif /* APDS_9301_H_ */