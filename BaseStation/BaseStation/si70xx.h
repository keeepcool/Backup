/*
 * si70xx.h
 *
 * Created: 13/05/2016 10:39:52
 *  Author: Tiago
 */ 


#ifndef SI70XX_H_
#define SI70XX_H_

#define F_CPU	16000000UL

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "pinMap.h"
#include "i2cmaster.h"
#include "uart.h"

// Addresses, registers, commands
#define SHT2x_I2CADDR               0x40 << 1
#define SHT2x_I2CADDR_WRITE         SHT2x_I2CADDR
#define SHT2x_I2CADDR_READ          SHT2x_I2CADDR|0x01
#define SHT2x_CMD_SOFT_RESET        0xFE
#define SHT2x_CMD_TEMPERATURE_HOLD  0xE3
#define SHT2x_CMD_HUMIDITY_HOLD     0xE5

// Function prototypes
extern void si70xx_init(void);
extern float si70xx_get_temperature(void);
extern float si70xx_get_humidity(void);



#endif /* SI70XX_H_ */