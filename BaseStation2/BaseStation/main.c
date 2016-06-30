/*
 * BaseStation.c
 *
 * Created: 21/04/2016 16:33:17
 * Author : Tiago
 */ 

#define F_CPU	16000000UL

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <stdint-gcc.h>

#include "wizchip_conf.h"
#include "pinMap.h"
#include "i2cmaster.h"
#include "socket.h"
#include "w5500.h"
#include "uart.h"
#include "si70xx.h"
#include "DS3231.h"

#include "sysConfig.h"
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "dns.h"
#include "dhcp.h"
#include "atmegarfr2.h"

//GLobal function declaration
void hsv2rgb(uint16_t hue, uint16_t sat, uint16_t val,uint8_t * r, uint8_t  * g, uint8_t  * b, uint8_t  maxBrightness);
int32_t clientTCP(uint8_t sn, uint8_t* destip, uint16_t destport);
uint8_t SPI_TransferByte(uint8_t data);
int8_t runDNSquery(uint8_t* domName);
static void W5500_Init(void);
void printNetInfo(void);
uint8_t greeting(void);
void spi_wb(uint8_t b);
void i2cScanner(void);
void TCP_Server(void);
void timer0Init(void);
void timer1Init(void);
uint64_t milis(void);
void sendNetInfo(void);
uint8_t spi_rb(void);
void cs_desel(void);
void ip_init(void);
void boardInit(void);
void ip_task(void);
uint8_t echo(void);
void cs_sel(void);
void printCSV(void);

void my_ip_conflict(void);
void my_ip_assign(void);
uint8_t dhcpRun(void);
void Net_Conf(void);
void dhcpInit(void);

typedef struct{
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	unsigned char amOrPM;
	unsigned char weekDay;
	unsigned char day;
	unsigned char month;
	unsigned char year;
} time_struct_t;

typedef struct{
	
	float airTemp;
	uint8_t airHR;
	float airPress;
	uint16_t lux;
	int8_t moveFlag;
	int16_t rawXaxis;
	int16_t rawYaxis;
	int16_t rawZaxis;
	uint16_t battVoltage;
	uint16_t sensorID;
	
}dataPacket_t;


dataPacket_t gDatapacket;
time_struct_t time;

//Global variables declaration
volatile uint64_t milisCount = 0;
volatile uint8_t globalDataFlag = 0;
volatile uint8_t newDatatoSend = 0;
uint64_t loopTime = 0;
uint64_t printMilis = 0;
uint64_t dhcpTimer = 0;

uint8_t Rpwm, Gpwm, Bpwm;
uint16_t hue_value = 0;

char gSendBuff[128];
char gTempDataBuff[128];
char *dataArray[16];
char* strPointer = 0;
int16_t sendRetVal = 0;

char gPtrintBuff[128];

//*********************************************************
//********* Start Net Config ******************************
//*********************************************************
// Socket & Port number definition
#define SOCK_ID_TCP			0
#define SOCK_ID_TCP_RECV	1
#define SOCK_DHCP			2
#define SOCK_DNS			3

#define MAX_DHCP_RETRY	2
uint8_t my_dhcp_retry = 0;

#define DATA_BUF_SIZE     1024
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t gSERVER_DATABUFF[DATA_BUF_SIZE];

//#define PORT_TCP          5000				//Client TCP Port
#define PORT_TCP_SERVER	  5215				//Server TCP Port

 wiz_NetInfo gWIZNETINFO =
{
	.mac = {0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF},	// Source Mac Address
	.ip = {192, 168,  1, 2},					// Source IP Address
	.sn = {255, 255, 255, 0},						// Subnet Mask
	.gw = {192, 168,  1, 1},					// Gateway IP Address
	.dns = {8, 8, 8, 8},					// DNS server IP Address
	.dhcp = NETINFO_DHCP
};

volatile wiz_PhyConf phyConf =
{
	PHY_CONFBY_HW,       // PHY_CONFBY_SW
	PHY_MODE_AUTONEGO,     // PHY_MODE_AUTONEGO
	PHY_SPEED_100,        // PHY_SPEED_100
	PHY_DUPLEX_FULL,     // PHY_DUPLEX_HALF
};

volatile wiz_NetInfo pnetinfo;
uint8_t dIP[4] = 	{192, 168, 35, 170};
uint16_t dport = 	5214;


uint8_t DNS_2nd[4] = {208, 67, 222, 222};
uint8_t Domain_name[] = "sensors.srv.ename.pt";
uint8_t Domain_IP[4] = {0,0,0,0};


//*********************************************************
//********* End Net Config ********************************
//*********************************************************

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

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
//static uint8_t appUartBuffer[APP_BUFFER_SIZE];
//static uint8_t appUartBufferPtr = 0;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req){
	
	if (NWK_SUCCESS_STATUS == req->status){
		
		uartPuts("\nFrame Sent\n");
		
	}
	
	
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void){
	
	if ((appDataReqBusy) || newDatatoSend == 0){
		return;
	}
	

	//memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

	appDataReq.dstAddr = 1-APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = appDataReqBuffer;
	appDataReq.size = sizeof(appDataReqBuffer);
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	//appUartBufferPtr = 0;
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
	
	/*
	uartPutsP("\nappDataInd\n");
	
	//Show link quality
	sprintf(gPtrintBuff, "RSSI: %i LQI: %i\n", ind->rssi, ind->lqi);
	uartPuts(gPtrintBuff);
	
	*/
	
	globalDataFlag = 0;
	
	for (uint8_t i = 0; i < ind->size; i++){
		gTempDataBuff[i] = ind->data[i];
	}
	
	globalDataFlag = 1;
	
	ind->size = 0;
	ind->data = 0;
	
	return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void){
	
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	PHY_SetRxState(1);
	PHY_SetTxPower(TX_PWR_0_5_DBM);
	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void){
	switch (appState)
	{
		case APP_STATE_INITIAL:
		{
			appInit();
			appState = APP_STATE_IDLE;
		} break;

		case APP_STATE_IDLE:
		break;

		default:
		break;
	}
}

/*************************************************************************//**
*****************************************************************************/
int main(void){
	
	time.amOrPM = PM;
	
	uint8_t iter = 0;
	
	boardInit();
	uartPutsP("Board Init Done\n");
	//i2cScanner();
	uartPutsP("Into Loop\n");
	
	//setTime( 14, 15, 20, amOrPm, t24_HOUR_FORMAT);
	//Its day of the week, day, month, year
	//setDate(5, 9, 6, 16);
	
	getTime(&time.hour, &time.min, &time.sec , &time.amOrPM, t24_HOUR_FORMAT);
	getDate(&time.weekDay, &time.day, &time.month, &time.year);
	
	sprintf(gPtrintBuff, "[Time set]> Time: %02i:%02i:%02i \n", time.hour, time.min, time.sec);
	uartPuts(gPtrintBuff);
	
	
	sprintf(gPtrintBuff, "[Date set]> Date: %02i:%02i:%02i \n", time.day, time.month, time.year);
	uartPuts(gPtrintBuff);
	
	
	
	
	dhcpInit();

	while(dhcpRun() != 10){
		
		if(milis() - loopTime >= 100){
			
			DHCP_time_handler();
			
			loopTime = milis();
			
		}
	}
	
	printNetInfo();
	
	//runDNSquery(Domain_name);
	
	
	if(runDNSquery("sensors.srv.ename.pt") > 0){
		dIP[0] = Domain_IP[0];
		dIP[1] = Domain_IP[1];
		dIP[2] = Domain_IP[2];
		dIP[3] = Domain_IP[3];
	}
	
	
	
	printNetInfo();
	
	while(getSn_SR(SOCK_ID_TCP) != SOCK_ESTABLISHED){
		clientTCP(SOCK_ID_TCP, dIP, dport);
	}
	
	sendNetInfo();
	
	while (1){
		
		
		if(milis() - dhcpTimer >= 100){
			
			//1 second time base for dhcp lease timer
			//DHCP_time_handler();
			dhcpTimer = milis();
			
		}
		
		if(milis() - loopTime >= 250){
			
			hsv2rgb(hue_value,255,80, &Rpwm, &Gpwm, &Bpwm, 250);
			OCR1A = Rpwm;
			OCR1B = Gpwm;
			OCR1C = Bpwm;
			hue_value++;
			if(hue_value > 359){
				hue_value = 0;
			}
			
			loopTime = milis();
			
		}
		

		if(milis() - printMilis >= 250){
			
			if(globalDataFlag == 1){
				uartPuts("\nReceived LWmesh: ");
				uartPuts(gTempDataBuff);
				
				sprintf(appDataReqBuffer, "OK\n");
				//appDataReqBuffer = "OK\n";
				newDatatoSend = 1;
				appSendData();
				
				iter = 0;
				strPointer = strtok(gTempDataBuff, ",");
				
				while(strPointer){
					dataArray[iter] = strPointer;
					iter++;
					strPointer = strtok(NULL, ",");
				}
				
				for(uint8_t j = 0; j<iter; j++){
					
					switch (dataArray[j][0])
					{
						
						case 'I':							//Sensor ID
						gDatapacket.sensorID = atoi(dataArray[j]+1);
						sprintf(gPtrintBuff, "I: %i \n", gDatapacket.sensorID);
						uartPuts(gPtrintBuff);
						break;
						
						case 'T':							//Air temperature
						gDatapacket.airTemp = atof(dataArray[j]+1);
						sprintf(gPtrintBuff, "T: %f \n", gDatapacket.airTemp);
						uartPuts(gPtrintBuff);
						break;
						
						case 'P':							//Air pressure
						gDatapacket.airPress = (atof(dataArray[j]+1)/100000.0);
						sprintf(gPtrintBuff, "P: %f \n", gDatapacket.airPress);
						uartPuts(gPtrintBuff);
						break;
						
						case 'L':							//Luminosity
						gDatapacket.lux = atoi(dataArray[j]+1);
						sprintf(gPtrintBuff, "L: %i \n", gDatapacket.lux);
						uartPuts(gPtrintBuff);
						break;
						case 'V':							//Sensor batt voltage
						gDatapacket.battVoltage = atof(dataArray[j]+1);
						sprintf(gPtrintBuff, "V: %i \n", gDatapacket.battVoltage);
						uartPuts(gPtrintBuff);
						break;
						
						case 'H':							//Air moisture
						gDatapacket.airHR = atoi(dataArray[j]+1);
						sprintf(gPtrintBuff, "H: %i \n", gDatapacket.airHR);
						uartPuts(gPtrintBuff);
						break;
						
						case 'A':							//Sensor batt voltage
						gDatapacket.moveFlag = atoi(dataArray[j]+1);
						sprintf(gPtrintBuff, "A: %i \n", gDatapacket.moveFlag);
						uartPuts(gPtrintBuff);
						break;
					}
				}
				
				getTime(&time.hour, &time.min, &time.sec , &time.amOrPM, t24_HOUR_FORMAT);
				getDate(&time.weekDay, &time.day, &time.month, &time.year);
				
				sprintf(gDATABUF, "D%i:%i:%i_%i:%i:%i I%i T%.1f H%i P%.3f L%i V%i", (time.year+2000), time.month, time.day, time.hour,time.min, time.sec, gDatapacket.sensorID, gDatapacket.airTemp, gDatapacket.airHR, gDatapacket.airPress, gDatapacket.lux, gDatapacket.battVoltage);
				uartPuts(gDATABUF);
				uartPutc('\n');
				
				sendRetVal = send(SOCK_ID_TCP, gDATABUF, strlen(gDATABUF));
				sprintf(gPtrintBuff, "ETH: Sent %i bytes\n", sendRetVal);
				uartPuts(gPtrintBuff);
				//uartPuts(gDATABUF);
				//uartPutc('\n');
				
				memset(gDATABUF, NULL, sizeof(gDATABUF));
				
				sendRetVal = recv(SOCK_ID_TCP,gDATABUF, sizeof(gDATABUF));
				uartPuts(gDATABUF);
				uartPutc('\n');
				
				//close(SOCK_ID_TCP);
				
				sprintf(gPtrintBuff, "ETH: Sent %i bytes\n", sendRetVal);
				uartPuts(gPtrintBuff);
				//uartPuts(gDATABUF);
				
				
				globalDataFlag = 0;
				
			}
			
			printMilis = milis();
			
		}
		
		/*
		float temperature = si70xx_get_temperature();
		float humidity = si70xx_get_humidity();
		*/
		
		//sprintf(gDATABUF, "%u | Temp: %.2f RH: %.2f \n", i , temperature, humidity);
		
		APP_TaskHandler();
		SYS_TaskHandler();
		
		//Handle the ethernet interface
		
		clientTCP(SOCK_ID_TCP, dIP, dport);
		TCP_Server();
		
		//serial "command" handler
		char temp = uartGetc();
		if(temp == 'S'){
			printNetInfo();
			
			
		}
		
	}
}


int8_t runDNSquery(uint8_t* domName){
	
	int8_t errVal = 0;
	
	//printNetInfo();
	//sendNetInfo();
	
	uartPutsP("\nStarting DNS query\n");

	sprintf(gPtrintBuff, "DNS Servers\n");
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff,"> DNS 1st : %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff,"> DNS 2nd : %d.%d.%d.%d\r\n", DNS_2nd[0], DNS_2nd[1], DNS_2nd[2], DNS_2nd[3]);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff,"=======================================\r\n");
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff,"> [Example] Target Domain Name : %s\r\n", Domain_name);
	uartPuts(gPtrintBuff);
	
	DNS_init(SOCK_DNS, gDATABUF);
	
	/* DNS processing */
	if ((errVal = DNS_run(gWIZNETINFO.dns, Domain_name, Domain_IP)) > 0){ // try to 1st DNS
		uartPutsP("> 1st DNS Respond\n");
	}
	
	else if ((errVal != -1) && ((errVal = DNS_run(DNS_2nd, Domain_name, Domain_IP))>0)){     // retry to 2nd DNS
		uartPutsP("> 2nd DNS Respond\n");
	}
	else if(errVal == -1){
		uartPutsP("> MAX_DOMAIN_NAME is too small. Should be redefined it.\n");
	}
	else{
		uartPutsP("> DNS Failed\n");
	}

	if(errVal > 0)
	{
		sprintf(gPtrintBuff, "> Translated %s to [%d.%d.%d.%d]\n",Domain_name,Domain_IP[0],Domain_IP[1],Domain_IP[2],Domain_IP[3]);
		uartPuts(gPtrintBuff);
	}
	
	return errVal;
	
	
}


uint8_t dhcpRun(void){
	
	uint8_t ipStatus = 0;
	
	if(gWIZNETINFO.dhcp == NETINFO_DHCP) {
		switch(DHCP_run())
		{
			case DHCP_IP_ASSIGN:
			uartPuts("\nDHCP is assigned\n");
			
			case DHCP_IP_CHANGED:
			uartPuts("\nDHCP changed \n");
			break;
			
			case DHCP_IP_LEASED:
			uartPuts("\nDHCP is leased\n");
			//printNetInfo();
			ipStatus = 10;
			//
			//
			break;
			case DHCP_FAILED:
			uartPuts("\nDHCP FAILED\n");
			/* ===== Example pseudo code =====  */
			// The below code can be replaced your code or omitted.
			// if omitted, retry to process DHCP
			my_dhcp_retry++;
			
			if(my_dhcp_retry > MAX_DHCP_RETRY){
				gWIZNETINFO.dhcp = NETINFO_STATIC;
				DHCP_stop();      // if restart, recall DHCP_init()
				sprintf(gPtrintBuff, ">> DHCP %d Failed\r\n", my_dhcp_retry);
				uartPuts(gPtrintBuff);
				Net_Conf();
				//printNetInfo();   // print out static netinfo to serial
				my_dhcp_retry = 0;
			}
			break;
			default:
			break;
		}
	}
	
	return ipStatus;
	
}

void dhcpInit(void) {/* DHCP client Initialization */
	if(gWIZNETINFO.dhcp == NETINFO_DHCP){
		uartPuts("\nDHCP Init\n");
		DHCP_init(SOCK_DHCP, gDATABUF);
		// if you want different action instead default ip assign, update, conflict.
		// if cbfunc == 0, act as default.
		uartPuts("\nCalling reg_dhcp\n");
		reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
	}
}

void my_ip_assign(void) {
   getIPfromDHCP(gWIZNETINFO.ip);
   getGWfromDHCP(gWIZNETINFO.gw);
   getSNfromDHCP(gWIZNETINFO.sn);
   getDNSfromDHCP(gWIZNETINFO.dns);
   gWIZNETINFO.dhcp = NETINFO_DHCP;
   
   /* Network initialization */
   Net_Conf();      // apply from DHCP
   //printNetInfo();
   sprintf(gPtrintBuff, "DHCP LEASE TIME : %ld Sec.\r\n", getDHCPLeasetime());
   uartPuts(gPtrintBuff);
   
}

/************************************
 * @ brief Call back for ip Conflict
 ************************************/
void my_ip_conflict(void) {
	
	uartPutsP("CONFLICT IP from DHCP\r\n");
   //halt or reset or any...
   while(1); // this example is halt.
}

 void Net_Conf(){
	/* wizchip netconf */
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

int32_t clientTCP(uint8_t sn, uint8_t* destip, uint16_t destport){
	
	int32_t ret; // return value for SOCK_ERRORs

	// Destination (TCP Server) IP info (will be connected)
	// >> loopback_tcpc() function parameter
	// >> Ex)
	//	uint8_t destip[4] = 	{192, 168, 34, 131};
	//	uint16_t destport = 	5000;

	// Port number for TCP client (will be increased)
	uint16_t any_port = 2;

	// Socket Status Transitions
	// Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
	switch(getSn_SR(sn))
	{
		case SOCK_ESTABLISHED :
		if(getSn_IR(sn) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
		{
			sprintf(gPtrintBuff, "%d:Connected to - %d.%d.%d.%d : %u\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
			uartPuts(gPtrintBuff);
			setSn_IR(sn, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
		}

		break;

		case SOCK_CLOSE_WAIT :
		if((ret=disconnect(sn)) != SOCK_OK) return ret;
		
			sprintf(gPtrintBuff, "%d:Socket Closed\r\n", sn);
			uartPuts(gPtrintBuff);
		break;

		case SOCK_INIT :
		
		sprintf(gPtrintBuff, "%d:Try to connect to the %d.%d.%d.%d : %u\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
		uartPuts(gPtrintBuff);
		
		if( (ret = connect(sn, destip, destport)) != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
		break;

		case SOCK_CLOSED:
		close(sn);
		if((ret=socket(sn, Sn_MR_TCP, any_port, 0x00)) != sn) return ret; // TCP socket open with 'any_port' port number
		
		sprintf(gPtrintBuff, "%d:TCP client started\r\n",sn);
		uartPuts(gPtrintBuff);
		sprintf(gPtrintBuff, "%d:Socket opened\r\n",sn);
		uartPuts(gPtrintBuff);
		break;
		default:
		break;
	}
	return 1;
}

void TCP_Server(void){
	int32_t ret;
	uint16_t size = 0, sentsize = 0;

	// Get status of socket
	switch(getSn_SR(SOCK_ID_TCP_RECV)) {
		
		// Connection established
		case SOCK_ESTABLISHED : {
			// Check interrupt: connection with peer is successful
			if(getSn_IR(SOCK_ID_TCP_RECV) & Sn_IR_CON) {
				// Clear corresponding bit
				setSn_IR(SOCK_ID_TCP_RECV,Sn_IR_CON);
			}

			// Get received data size
			if((size = getSn_RX_RSR(SOCK_ID_TCP_RECV)) > 0) {
				// Cut data to size of data buffer
				if(size > DATA_BUF_SIZE) {
					
					size = DATA_BUF_SIZE;
				}

				// Get received data
				ret = recv(SOCK_ID_TCP_RECV, gSERVER_DATABUFF, size);
				
				if(ret > 0 ){
					
					uartPuts("\nCLIENT SAYS: ");
					uartPuts(gSERVER_DATABUFF);					
				}
				
				// Check for error
				if(ret <= 0){
					return;
				}

				// Send echo to remote
				sentsize = 0;
				while(size != sentsize)
				{
					ret = send(SOCK_ID_TCP_RECV, gSERVER_DATABUFF + sentsize, size - sentsize);
					
					// Check if remote close socket
					if(ret < 0)
					{
						close(SOCK_ID_TCP_RECV);
						return;
					}

					// Update number of sent bytes
					sentsize += ret;
				}
			}
			break;
		}

		// Socket received the disconnect-request (FIN packet) from the connected peer
		case SOCK_CLOSE_WAIT : {
			// Disconnect socket
			if((ret = disconnect(SOCK_ID_TCP_RECV)) != SOCK_OK)
			{
				return;
			}

			break;
		}

		// Socket is opened with TCP mode
		case SOCK_INIT :{
			
			// Listen to connection request
			if( (ret = listen(SOCK_ID_TCP_RECV)) != SOCK_OK){
				return;
			}

			break;
		}

		// Socket is released
		case SOCK_CLOSED:{
			
			// Open TCP socket
			if((ret = socket(SOCK_ID_TCP_RECV, Sn_MR_TCP, PORT_TCP_SERVER, 0x00)) != SOCK_ID_TCP){
				return;
			}

			break;
		}

		default:{
			
			break;
		}
	}
}


void i2cScanner(void){
	
	uartPutsP("\n[Starting I2C scan]\n");
	uartPutsP(">Enabling Pull-up's\n");
	uint8_t retVal;
	
	for(int i=0; i<120; i++){
		
		sprintf(gPtrintBuff, ">Found: %x\r", i);
		retVal = 2;
		retVal = i2c_start(i<<1);
		_delay_ms(1);
		i2c_stop();
		_delay_ms(10);
		
		if(retVal == 0){
			sprintf(gPtrintBuff, ">Found: %x\r", i);
			uartPuts(gPtrintBuff);
		}
	}
	
	uartPutsP("[I2C Scan Complete!]\n");
}

void sendNetInfo(void){
	
	int16_t error = 0;
	wiz_NetInfo readBack;
	
	wizchip_getnetinfo(&readBack);
	
	sprintf(gDATABUF, "IP %d.%d.%d.%d", readBack.ip[0], readBack.ip[1], readBack.ip[2], readBack.ip[3]);
	uartPuts(gDATABUF);
	uartPutc('\n');
	error = send(SOCK_ID_TCP, gDATABUF, strlen(gDATABUF));
	clientTCP(SOCK_ID_TCP, dIP, dport);
	sprintf(gPtrintBuff, "Send error: %i \n", error);
	uartPuts(gPtrintBuff);
}

void printNetInfo(void){
	
	wiz_NetInfo readBack;
	
	uint8_t version = getVERSIONR();
	wizchip_getnetinfo(&readBack);
	if(readBack.dhcp == NETINFO_DHCP){
		sprintf(gPtrintBuff, "\n=== NET CONF : DHCP ===\n");
		uartPuts(gPtrintBuff);
	}
	else {
		sprintf(gPtrintBuff, "\n=== NET CONF : STATIC ===\n");
		uartPuts(gPtrintBuff);
	}
	sprintf(gPtrintBuff, "====== HW ID : %i ======\n", version);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff, " MAC : %02X:%02X:%02X:%02X:%02X:%02X\n", readBack.mac[0], readBack.mac[1], readBack.mac[2], readBack.mac[3], readBack.mac[4], readBack.mac[5]);
	uartPuts(gPtrintBuff);	
	sprintf(gPtrintBuff, " IP : %d.%d.%d.%d\n", readBack.ip[0], readBack.ip[1], readBack.ip[2], readBack.ip[3]);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff, " GW : %d.%d.%d.%d\n", readBack.gw[0], readBack.gw[1], readBack.gw[2], readBack.gw[3]);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff, " SN : %d.%d.%d.%d\n", readBack.sn[0], readBack.sn[1], readBack.sn[2], readBack.sn[3]);
	uartPuts(gPtrintBuff);
	sprintf(gPtrintBuff, "=======================================\r\n");
	uartPuts(gPtrintBuff);
	
}

void boardInit(void){
	
	PMOD_DDR |= _BV(PMOD0) | _BV(PMOD1) | _BV(PMOD2) | _BV(PORTD4);		//Set PMOD as outputs
	WIZ_DDR &= ~(_BV(INT_W5500));							//Int is an input
	WIZ_PORT |= _BV(INT_W5500);								//Enable pull up
	WIZ_DDR |= _BV(SS_W5500);								//Set SS as output 
	AUX_DDR &= ~(_BV(BTN0) | _BV(ISENSE) | _BV(VSENSE));	//Button set as input Isense and Vsense inputs(will be used for ADC)
	AUX_DDR |= _BV(WP_EEP);									//WP pin for eeprom as output
	LED_DDR |= _BV(LED_R) | _BV(LED_G) | _BV(LED_B);
	AUX_PORT |= _BV(BTN0);									//Enable pull up on btn
	
	PMOD_PORT |= _BV(PMOD0) | _BV(PMOD1) | _BV(PMOD2);		//Full auto
	
	
	uartInit();
	i2c_init();
	sei();
	
	DDRB |= _BV(PORTB2) | _BV(PORTB1) | _BV(PORTB0);
	
	SPCR = (1 << SPE)  | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1);
	SPSR = (1<<SPI2X);
	
	uartPutsP("\n\n[Base Station]\n");
	uartPutsP("> Pins init\n");
	timer0Init();
	uartPutsP("> Timer 0 init\n");
	timer1Init();
	uartPutsP("> Timer 1 init\n");
	DS3231_init();
	uartPutsP("> DS3231(RTC) init\n");
	W5500_Init();
	_delay_ms(10);
	uartPutsP("> W5500 init\n");
	sprintf(gPtrintBuff, "> W5500 version: %u\n",getVERSIONR());
	uartPuts(gPtrintBuff);
	si70xx_init();
	uartPutsP("> si70xx init\n");
	SYS_Init();
	
	//_delay_ms(1000);
	

}

void timer0Init(void){
	//Used for 10ms time base
	
	TCCR0A = _BV(WGM01);	//CTC mode, top = OCR0A
	OCR0A = 156;	//For 10ms time base
	TCCR0B = _BV(CS00) | _BV(CS02);	//Clkio:1024
	TIMSK0 = _BV(OCIE0A);
	
}

void timer1Init(void){
	//Used for RGB led 
	// R is OC1A
	// G is OC1B
	// B is OC1C
	
	TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1)| _BV(COM1B0) | _BV(COM1C1)| _BV(COM1C0) | _BV(WGM10); //PWM phase correct 8 bits
	TCCR1B = _BV(CS10) | _BV(CS11);	//Clkio:8 clock
	OCR1A = 40;
	OCR1B = 10;
	OCR1C = 100;
}

// brief Initialize modules
static void W5500_Init(void){
	// Set Tx and Rx buffer size to 2KB
	uint8_t buffsize[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };

	// Reset module
	PMOD_PORT &= ~_BV(RST_W5500);		//Set Rst low
	_delay_ms(500);						//Minimun of 500us, 10ms to be safe
	PMOD_PORT |= _BV(RST_W5500);		//Set rst high
	_delay_ms(1);


	// Wizchip initialize
	wizchip_init(buffsize, buffsize, 0, 0, cs_sel, cs_desel, 0, 0, spi_rb, spi_wb);
	
	// Wizchip netconf
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
	ctlwizchip(CW_SET_PHYCONF, (void*) &phyConf);
}

uint8_t SPI_TransferByte (uint8_t data){
	
	SPDR = data;
	while(!(SPSR & (1<<SPIF))){
	}
	return SPDR;
	
	
}
void cs_sel(void){
	//CS LOW
	WIZ_PORT &= ~(_BV(SS_W5500));
}

void cs_desel(void){
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS HIGH
	WIZ_PORT |= (_BV(SS_W5500));
}

uint8_t spi_rb(void){
	return SPI_TransferByte(0x00);
}

void spi_wb(uint8_t b){
	SPI_TransferByte(b);
}

void hsv2rgb(uint16_t hue, uint16_t sat, uint16_t val,uint8_t * r, uint8_t  * g, uint8_t  * b, uint8_t  maxBrightness ) {
	uint16_t H_accent = hue/60;
	uint16_t bottom = ((255 - sat) * val)>>8;
	uint16_t top = val;
	uint8_t  rising  = ((top-bottom)  *(hue%60   )  )  /  60  +  bottom;
	uint8_t  falling = ((top-bottom)  *(60-hue%60)  )  /  60  +  bottom;
	
	switch(H_accent) {
		case 0:
		*r = top;
		*g = rising;
		*b = bottom;
		break;
		
		case 1:
		*r = falling;
		*g = top;
		*b = bottom;
		break;
		
		case 2:
		*r = bottom;
		*g = top;
		*b = rising;
		break;
		
		case 3:
		*r = bottom;
		*g = falling;
		*b = top;
		break;
		
		case 4:
		*r = rising;
		*g = bottom;
		*b = top;
		break;
		
		case 5:
		*r = top;
		*g = bottom;
		*b = falling;
		break;
	}
	// Scale values to maxBrightness
	*r = *r * maxBrightness/255;
	*g = *g * maxBrightness/255;
	*b = *b * maxBrightness/255;
}

uint64_t milis(void){
	
	cli();
	uint64_t safeMilis = milisCount;
	sei();
	
	return safeMilis;
}


//Interrupts go after this

ISR(TIMER0_COMPA_vect){
	
	milisCount++;
	
}