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
uint16_t adcRead(uint8_t channel);
void adcInit(void);


/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

#define VREF			(1800.0f)
#define ADC_CNTS		(1023.0f)

//Function declaration
void timer3Init(void);
void i2cScanner(void);
uint64_t getMilis(void);

//ISR interrupt vars
volatile uint64_t millisCount = 0;

//Global var declaration
uint64_t printMilis = 0;
uint64_t dataMilis = 0;
int16_t intLux = 0;
uint8_t var=12;
int8_t dir = 1;

int16_t axisData[3];
float axisFloat[3];

char buff[256];
char gPrintBuff[256];
char gTempDataBuff[256];

float FLT_CONST = (float)(VREF/ADC_CNTS);

typedef struct dataPacket_t
{
	
	float airTemp;
	uint8_t airHR;
	float airPress;
	uint16_t lux;
	int8_t moveFlag;
	int16_t rawXaxis;
	int16_t rawYaxis;
	int16_t rawZaxis;
	uint16_t battVoltage;
	
}dataPacket;


typedef struct sysConf_t{
	
	uint16_t sendInterval;	//Value in seconds between data packets
	uint16_t selfID;
	int8_t sendRawAcc;		//Enable/disable raw acc axis data
	
}sysConf;


dataPacket gDataPacket = {
	.airTemp = 0,
	.airHR = 0,
	.airPress = 0,
	.lux = 0,
	.moveFlag = 0,
	.rawXaxis = 0,
	.rawYaxis = 0,
	.rawZaxis = 0,
	.battVoltage = 0	
};



sysConf gSysConf = {
	
	.sendInterval = 1,
	.selfID = 20,
	.sendRawAcc = 0
};



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
static void appDataConf(NWK_DataReq_t *req){
	
	if (NWK_SUCCESS_STATUS == req->status){
		
		if(NWK_SUCCESS_STATUS == req->status){
			
			uartPuts("\nFRAME SENT\n");
		}
		
	}	
	
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void){
	
	if (appDataReqBusy || newDataTosend == 0){
			return;
	}
	
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
	
	sprintf(gPrintBuff, "Received :%i\n", ind->size);
	uartPuts(gPrintBuff);
	
	for (uint8_t i = 0; i < ind->size; i++){
		//uartPutc(ind->data[i]);
		gTempDataBuff[i] = ind->data[i];
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
	//LED_DDR &= ~_BV(LED_PIN);
	CTRL_DDR &= ~_BV(PLEN_PIN);
	CTRL_DDR |= _BV(PLEN_PIN);
	CTRL_PORT |= _BV(PLEN_PIN);	//Active low, start with pull up disabled
	uartInit();
	sei();
	
	timer3Init();
	i2c_init();
	SYS_Init();
	adcInit();
	adcRead(2);
	
	sei();
	
	APDS_Init();
	BME280_Init(FIRST_INIT);
	//MMA8652_printID();
	MMA8652_Init(RANGE_2G);
	
	uartPutsP("\n[Init Done]\n");
	
	//i2cScanner();
	
    while (1){
		
		SYS_TaskHandler();
		APP_TaskHandler();
		
		/*
		if(getMilis() - printMilis >= 100){
			
			
			printMilis = getMilis();
		}
		*/
		
		
		if(getMilis() - dataMilis > (gSysConf.sendInterval * 1000)){
			
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
			BME280_Init(0);
			gDataPacket.airHR = BME280_readFloatHumidity();
			gDataPacket.airPress = BME280_readFloatPressure();
			gDataPacket.airTemp = BME280_readTempC();
			gDataPacket.lux = APDS_getLux();
			gDataPacket.battVoltage = adcRead(2)*(FLT_CONST*4);
			if(gSysConf.sendRawAcc){
				MMA8652_readAcc(axisData);
				gDataPacket.rawXaxis = axisData[0];
				gDataPacket.rawYaxis = axisData[1];
				gDataPacket.rawZaxis = axisData[2];
			}
			
			sprintf(appDataReqBuffer, "I%i,P%.1f,H%i,T%.1f,L%i,A%i,V%i\n", gSysConf.selfID, gDataPacket.airPress, gDataPacket.airHR, gDataPacket.airTemp, gDataPacket.lux, gDataPacket.moveFlag, gDataPacket.battVoltage);
			uartPuts(appDataReqBuffer);
			sprintf(gPrintBuff, "String length: %i\n",strlen(appDataReqBuffer));
			uartPuts(gPrintBuff);
			
			
			
			newDataTosend = 1;
			appSendData();
			
			uartPuts(gTempDataBuff);
			
			dataMilis = getMilis();
			
		}
		
		
		
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

void adcInit(void){

	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));	//16Mhz/128 = 125Khz
	ADMUX |= (0<<REFS1)|(1<<REFS0);				//Referencia de 5v
	ADMUX &= ~(1<<REFS1);
	ADCSRC |= _BV(ADTHT1);
	ADCSRA |= (1<<ADEN);				//Adc ligada
	ADCSRA |= (1<<ADSC);				//Fazer uma primeira conversão para iniciar o circuito e porque é a mais lenta
}

uint16_t adcRead(uint8_t channel){
	ADMUX &= 0xE0;						//Limpa o canal anterior
	ADMUX |= channel;					//Define o novo canal a ler do ADC
	ADCSRA |= (1<<ADSC);				//Inicia uma nova conversão
	while(ADCSRA & (1<<ADSC));			//Espera que a conversão seja feita
	return (ADCW);						//Retorna o valor do ADC
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
