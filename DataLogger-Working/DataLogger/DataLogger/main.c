/*
 * DataLogger.c
 *
 * Created: 20/01/2016 16:50:49
 * Author : Tiago
 */ 
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
#include "pinMap.h"
#include "i2cmaster.h"
#include "uart.h"
#include "BME280.h"
#include "APDS_9301.h"
#include "MMA8652FC.h"
#include "atmegarfr2.h"

#include "sysConfig.h"
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
//#include "halUart.h"
#include "halBoard.h"


/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif


//Function declaration
void timer3Init(void);
void i2cScanner(void);
void dataBuilder(void);
uint64_t getMilis(void);

//ISR interrupt vars
volatile uint64_t millisCount = 0;

//Global var declaration
uint64_t printMilis = 0;
uint64_t ledMilis = 0;
int16_t intLux = 0;
uint8_t var=12;
int8_t dir = 1;

int16_t axisData[3];
float axisFloat[3];

char buff[64];


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
	APP_STATE_SLEEP,
	APP_STATE_PREPARE_TO_SLEEP,
	APP_STATE_WAKEUP

} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
//static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t newDataTosend = 0;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void){
	if (appDataReqBusy || newDataTosend == 0){
			return;
	}
	
	uartPutsP("\nSending\n");

	appDataReq.dstAddr = DEST_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = appDataReqBuffer;
	appDataReq.size = strlen(appDataReqBuffer);
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	newDataTosend = 0;
	appDataReqBusy = true;
	
	SYS_TimerStop(&appTimer);
	SYS_TimerStart(&appTimer);
	
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer){
	appSendData();
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind){
	for (uint8_t i = 0; i < ind->size; i++){
		//uartPutc(ind->data[i]);
	}
	return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void){

	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	PHY_SetTxPower(TX_PWR_0_5_DBM);

	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void){
	switch (appState) {
		
		case APP_STATE_INITIAL: {
			appInit();
			appState = APP_STATE_IDLE;
		}
		break;
		
		case APP_STATE_PREPARE_TO_SLEEP: {
			if (!NWK_Busy()){
				
				NWK_SleepReq();
				appState = APP_STATE_SLEEP;
			}
		}
		break;
		
		case APP_STATE_WAKEUP: {
			NWK_WakeupReq();
			appState = APP_STATE_IDLE;

		}
		break;

		case APP_STATE_IDLE:
		break;

		default:
		break;
		
	}
}


int main(void){
    
	/* Replace with your application code */
	
	LED_DDR |= _BV(LED_PIN);
	LED_DDR &= ~_BV(LED_PIN);
	CTRL_DDR &= ~_BV(PLEN_PIN);
	CTRL_DDR |= _BV(PLEN_PIN);
	CTRL_PORT |= _BV(PLEN_PIN);	//Active low, start with pull up disabled
	uartInit();
	sei();
	
	
	
	uartPutsP("\nUART INIT\n");
	timer3Init();
	i2c_init();
	
	SYS_Init();
	//HAL_UartInit(38400);
	sei();
	
	
	uartPutsP("\n[Init Done]\n");
	
	APDS_Init();
	BME280_Init(FIRST_INIT);
	MMA8652_printID();
	MMA8652_Init(RANGE_2G);
	
	//i2cScanner();
	
    while (1){
		
		SYS_TaskHandler();
		APP_TaskHandler();
		
		//sprintf(buff, "[Lux: %f]\n", APDS_getLux());
		//sprintf(buff, "%f\n", APDS_getLux());
		//uartPuts(buff);
		//LED_PORT ^= _BV(LED_PIN);
		
		if(getMilis() - printMilis >= 100){
			
			
			//sprintf(buff, "X: %i Y: %i Z: %i\n", axisData[0], axisData[1], axisData[2]);
			//sprintf(buff, "%i,%i,%i\n", axisData[0], axisData[1], axisData[2]);
			
			/*
			sprintf(buff, "X: %1.3f, Y: %1.3f, Z: %1.3f\n", axisFloat[0], axisFloat[1], axisFloat[2]);
			uartPuts(buff);
			*/
			printMilis = getMilis();
		}
		
		
		if(getMilis() - ledMilis > 250){
			
			/*	
			OCR3A=var;
			var += dir;
			
			if(var >= 220){
				dir= dir*(-1);
			}
			
			if(var <= 12){
				dir = dir *(-1);
			}
			
			*/
			
			//MMA8652_readAccG(axisFloat);
			//sprintf(appDataReqBuffer, "X: %i Y: %i Z: %i\n\0", axisData[0], axisData[1], axisData[2]);
			//uartPuts(appDataReqBuffer);
			
			while(BME280_IsMeasuring());
			dataBuilder();
			BME280_Init(0);
			MMA8652_readAcc(axisData);
			intLux = APDS_getLux();
			
			newDataTosend = 1;
			
			ledMilis = getMilis();
			
		}
		
		appSendData();
		
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

void dataBuilder(void){
	
	//char Lbuff[64];
	sprintf(appDataReqBuffer, "%4.1f,%4.1f,%4.1f,%4.1f,%i\n", BME280_readTempC(), BME280_readFloatPressure(), BME280_readFloatAltitudeMeters(), BME280_readFloatHumidity(), intLux);
	uartPuts(appDataReqBuffer);

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

ISR(TIMER3_COMPB_vect){
	
	millisCount++;
	
}
