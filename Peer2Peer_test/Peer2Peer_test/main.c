/*
 * Peer2Peer_test.c
 *
 * Created: 03/03/2016 10:35:01
 * Author : Tiago
 */ 

#include <avr/io.h>

/**
 * \file Peer2Peer.c
 *
 * \brief Peer2Peer application implementation
 *
 * Copyright (C) 2012-2014, Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 * Modification and other use of this code is subject to Atmel's Limited
 * License Agreement (license.txt).
 *
 * $Id: Peer2Peer.c 9267 2014-03-18 21:46:19Z ataradov $
 *
 */

/*- Includes ---------------------------------------------------------------*/
#define F_CPU	16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
//#include "halBoard.h"
#include "uart.h"

#include "pinMap.h"
#include "i2cmaster.h"
#include "uart.h"
#include "BME280.h"
#include "APDS_9301.h"
#include "MMA8652FC.h"

/*- Prototypes -------------------------------------------------------------*/
static void appTimerHandler(SYS_Timer_t *timer);
static void appDataConf(NWK_DataReq_t *req);
void HAL_UartBytesReceived(uint16_t bytes);
static bool appDataInd(NWK_DataInd_t *ind);
static void APP_TaskHandler(void);
static void appSendData(void);
static void appSendData(void);
static void appInit(void);
uint64_t getMilis(void);
void timer3Init(void);
void i2cScanner(void);
void printCSV(void);

/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;


/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;

volatile uint64_t millisCount = 0;
uint64_t printMilis = 0;
uint64_t ledMilis = 0;
uint8_t var=12;
int8_t dir = 1;
int16_t axisData[3];
float axisFloat[3];


char buff[64];


int main(void){
	
	uint16_t byteCount = 0;
	char buffer0[32];
	
	HAL_Init();
	SYS_TimerInit();
	PHY_Init();
	NWK_Init();
	//SYS_INIT()
	timer3Init();
	uartInit();
	sei();

	while (1){
		
		SYS_TaskHandler();
		
		byteCount = uartAvailable();
		
		if(byteCount > 0){
			HAL_UartBytesReceived(byteCount);
			sprintf(buffer0, "Bytes to send: %i\n", byteCount);
			uartPuts(buffer0);
		}
		APP_TaskHandler();
	}
}


void initSleep(void){
	
	//Sleep modes, lowest 0.9mA
	//TRXPR = 1 << SLPTR;
	//PRR0 = _BV(PRPGA) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRADC) | _BV(PRUSART0);
	//PRR1 = _BV(PRTRX24) | _BV(PRUSART1) | _BV(PRTIM3) | _BV(PRTIM4) | _BV(PRTIM5);
	//DRTRAM0 = _BV(ENDRT);
	//DRTRAM1 = _BV(ENDRT);
	//DRTRAM2 = _BV(ENDRT);
	//DRTRAM3 = _BV(ENDRT);
	
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN); // select power down mode
	//sleep_enable();
	//sleep_cpu(); // go to deep sleep
	//sleep_disable(); // executed after wake-
	
}

uint64_t getMilis(void){
	
	cli();
	uint64_t copy = millisCount;
	sei();
	
	return copy;
	
}

void printCSV(void){
	
	char Lbuff[64];
	sprintf(Lbuff, "%4.2f,%4.2f,%4.2f,%4.2f\n", BME280_readTempC(), BME280_readFloatPressure(), BME280_readFloatAltitudeMeters(), BME280_readFloatHumidity());
	uartPuts(Lbuff);

}


void timer3Init(void){
	//For some led(OC3A) pwm tests
	TCCR3A = _BV(COM3A1)|_BV(COM3A0)|_BV(WGM30);	//PWM, Phase correct, 8 bits ?
	TCCR3B = _BV(CS31)|_BV(CS30);					//Prescaller 1:61
	OCR3B = 250;	//For 1ms interrupts
	TIMSK3 = _BV(OCIE3B);
}



void i2cScanner(void){
	
	uartPutsP("\n[Starting I2C scan]\n");
	uartPutsP(">Enabling Pull-up's\n");
	CTRL_PORT &= ~_BV(PLEN_PIN);
	uint8_t retVal;
	
	for(int i=0; i<120; i++){
		
		sprintf(buff, ">Found: %x\r", i);
		retVal = 2;
		retVal = i2c_start(i<<1);
		_delay_ms(1);
		i2c_stop();
		_delay_ms(10);
		
		if(retVal == 0){
			sprintf(buff, ">Found: %x\r", i);
			uartPuts(buff);
		}
	}
	
	uartPutsP("[I2C Scan Complete!]\n");
}

static void appDataConf(NWK_DataReq_t *req){
	appDataReqBusy = false;
	(void)req;
}

static void appSendData(void){
	if (appDataReqBusy || 0 == appUartBufferPtr)
	return;

	memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

	appDataReq.dstAddr = 1-APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = appDataReqBuffer;
	appDataReq.size = appUartBufferPtr;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
}

static void appTimerHandler(SYS_Timer_t *timer){
	appSendData();
	(void)timer;
}

static bool appDataInd(NWK_DataInd_t *ind){
	
	for (uint8_t i = 0; i < ind->size; i++){
		uartPutc((char)ind->data[i]);
	}
	return true;
}

static void appInit(void){
	
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}

static void APP_TaskHandler(void){
	switch (appState){
		case APP_STATE_INITIAL:{
			appInit();
			appState = APP_STATE_IDLE;
		} break;

		case APP_STATE_IDLE:
		break;

		default:
		break;
	}
}

void HAL_UartBytesReceived(uint16_t bytes){

	for (uint16_t i = 0; i < bytes; i++){
		uint8_t byte = uartGetc();

		if (appUartBufferPtr == sizeof(appUartBuffer))
		appSendData();

		if (appUartBufferPtr < sizeof(appUartBuffer))
		appUartBuffer[appUartBufferPtr++] = byte;
	}

	SYS_TimerStop(&appTimer);
	SYS_TimerStart(&appTimer);
}


ISR(TIMER3_COMPB_vect){

	millisCount++;

}