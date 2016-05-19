/*
 * BaseStation.c
 *
 * Created: 21/04/2016 16:33:17
 * Author : Tiago
 */ 

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

#include "wizchip_conf.h"
#include "pinMap.h"
#include "i2cmaster.h"
#include "socket.h"
#include "w5500.h"
#include "uart.h"
#include "si70xx.h"

#define GREETING_MSG 		 "Bye bye!\r\n"

//GLobal function declaration
void hsv2rgb(uint16_t hue, uint16_t sat, uint16_t val,uint8_t * r, uint8_t  * g, uint8_t  * b, uint8_t  maxBrightness);
uint8_t SPI_TransferByte(uint8_t data);
static void W5500_Init(void);
void printNetInfo(void);
uint8_t greeting(void);
void spi_wb(uint8_t b);
void i2cScanner(void);
void timer0Init(void);
void timer1Init(void);
uint64_t milis(void);
uint8_t spi_rb(void);
void cs_desel(void);
void ip_init(void);
void sysInit(void);
void ip_task(void);
uint8_t echo(void);
void cs_sel(void);


int32_t loopback_tcpc(uint8_t sn, uint8_t* destip, uint16_t destport);

//Global variables declaration
volatile uint64_t milisCount = 0;

uint64_t loopTime = 0;
uint8_t Rpwm, Gpwm, Bpwm;
uint16_t hue_value = 0;

char gPtrintBuff[64];

// Socket & Port number definition
#define SOCK_ID_TCP       3
#define SOCK_ID_UDP       4

#define PORT_UDP          10001

#define DATA_BUF_SIZE     1024
uint8_t gDATABUF[DATA_BUF_SIZE];

//Server IP
#define PORT_TCP          5005
volatile wiz_NetInfo gWIZNETINFO =
{
	{0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF},    // Source Mac Address
	{192, 168,  34, 252},                      // Source IP Address
	{255, 255, 252, 0},                      // Subnet Mask
	{192, 168,  34, 131},                       // Gateway IP Address
	{8, 8, 8, 8},                      // DNS server IP Address
	NETINFO_STATIC
};

volatile wiz_PhyConf phyConf =
{
	PHY_CONFBY_HW,       // PHY_CONFBY_SW
	PHY_MODE_MANUAL,     // PHY_MODE_AUTONEGO
	PHY_SPEED_10,        // PHY_SPEED_100
	PHY_DUPLEX_FULL,     // PHY_DUPLEX_HALF
};

volatile wiz_NetInfo pnetinfo;

//****************************************************

uint8_t dIP[4] = 	{192, 168, 35, 170};
uint16_t dport = 	5000;

uint16_t i = 0;
uint8_t gSendBuff[256];
uint16_t retVal = 0;

int main(void){
	
	sysInit();
	
	SPCR = (1 << SPE)  | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1);
	SPSR = (1<<SPI2X);
	W5500_Init();
	
	_delay_ms(10);
	sprintf(gPtrintBuff, "Version: %u\n",getVERSIONR());
	uartPuts(gPtrintBuff);
	//i2cScanner();
	
	si70xx_init();
	
	uartPutsP("\nInto Loop\n");
	
    while (1){
		
		if(milis() - loopTime >= 50){
			
			hsv2rgb(hue_value,255,80, &Rpwm, &Gpwm, &Bpwm, 250);			
			OCR1A = Rpwm;
			OCR1B = Gpwm;
			OCR1C = Bpwm;
			hue_value++;
			if(hue_value > 359){
				hue_value = 0;
			}
			
			float temperature = si70xx_get_temperature();
			float humidity = si70xx_get_humidity();
			
			/*
			while(getSn_SR(SOCK_ID_TCP) != SOCK_ESTABLISHED){
				loopback_tcpc(SOCK_ID_TCP, dIP, dport);
			}
			*/
			
			sprintf(gDATABUF, "%u | Temp: %.2f RH: %.2f \n\0", i , temperature, humidity);
			
			retVal = send(SOCK_ID_TCP, gDATABUF, sizeof(gDATABUF));
			//close(SOCK_ID_TCP);
			
			sprintf(gPtrintBuff, "%u | Sent: %i Sizeof: %u\n", i ,  retVal, sizeof(gDATABUF));
			uartPuts(gPtrintBuff);
			
			i++;
			
			loopTime = milis();
			
		}
		
		
		loopback_tcpc(SOCK_ID_TCP, dIP, dport);
		//send(TCP_)
		char temp = uartGetc();
		if(temp == 'S'){
			printNetInfo();
			//uartPutc(temp);
		}
		
		if(temp == 's'){
			send(SOCK_ID_TCP, 'a', sizeof('a'));
			//uartPutc(temp);
		}
    }
}


int32_t loopback_tcpc(uint8_t sn, uint8_t* destip, uint16_t destport){
	
	int32_t ret; // return value for SOCK_ERRORs

	// Destination (TCP Server) IP info (will be connected)
	// >> loopback_tcpc() function parameter
	// >> Ex)
	//	uint8_t destip[4] = 	{192, 168, 34, 131};
	//	uint16_t destport = 	5000;

	// Port number for TCP client (will be increased)
	uint16_t any_port = 	2;

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
		if((ret=socket(sn, Sn_MR_TCP, any_port++, 0x00)) != sn) return ret; // TCP socket open with 'any_port' port number
		
		sprintf(gPtrintBuff, "%d:TCP client loopback start\r\n",sn);
		uartPuts(gPtrintBuff);
		sprintf(gPtrintBuff, "%d:Socket opened\r\n",sn);
		uartPuts(gPtrintBuff);
		break;
		default:
		break;
	}
	return 1;
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
	//readBack.dns
	sprintf(gPtrintBuff, "=======================================\r\n");
	uartPuts(gPtrintBuff);
	
}

void sysInit(void){
	
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
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);  // SPI enable, Master, f/16
	
	uartPutsP("\n\n[Base Station]\n");
	uartPutsP(">Pins init\n");
	timer0Init();
	uartPutsP(">Timer 0 init\n");
	timer1Init();
	uartPutsP(">Timer 1 init\n");

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

/*
void TCP_Client(void){
	int32_t ret;
	uint16_t size = 0, sentsize = 0;
	int16_t funcVal  = 0;


	uartPutsP("\nTCP_Client\n");
	// Get status of socket
	funcVal = getSn_SR(SOCK_ID_TCP);
	//sprintf(gPtrintBuff, "\ngetSn_SR(%i):  %#02x\n", SOCK_ID_TCP, funcVal);
	//uartPuts(gPtrintBuff);
	
	switch(funcVal){
		// Connection established
		case SOCK_ESTABLISHED :{
			//uartPutsP("\nScoket estabilished\n");
			// Check interrupt: connection with peer is successful
			if(getSn_IR(SOCK_ID_TCP) & Sn_IR_CON)
			{
				// Clear corresponding bit
				setSn_IR(SOCK_ID_TCP,Sn_IR_CON);
				//uartPutsP("\nConnection Successful\n");
			}

			// Get received data size
			if((size = getSn_RX_RSR(SOCK_ID_TCP)) > 0)
			{
				// Cut data to size of data buffer
				if(size > DATA_BUF_SIZE)
				{
					size = DATA_BUF_SIZE;
				}

				// Get received data
				ret = recv(SOCK_ID_TCP, gDATABUF, size);

				// Check for error
				if(ret <= 0)
				{
					return;
				}

				// Send echo to remote
				sentsize = 0;
				while(size != sentsize)
				{
					ret = send(SOCK_ID_TCP, gDATABUF + sentsize, size - sentsize);
					
					// Check if remote close socket
					if(ret < 0)
					{
						close(SOCK_ID_TCP);
						return;
					}

					// Update number of sent bytes
					sentsize += ret;
				}
			}
			break;
		}

		// Socket received the disconnect-request (FIN packet) from the connected peer
		case SOCK_CLOSE_WAIT :
		{
			// Disconnect socket
			if((ret = disconnect(SOCK_ID_TCP)) != SOCK_OK)
			{
				
				
				return;
			}

			break;
		}

		// Socket is opened with TCP mode
		case SOCK_INIT :{
			//uartPutsP("SOCK_INIT\n");
			// Listen to connection request
			if( (ret = connect(SOCK_ID_TCP, addrT, PORT_TCP)) != SOCK_OK){

				return;
			}

			break;
		}

		// Socket is released
		case SOCK_CLOSED:
		{
			// Open TCP socket
			if((ret = socket(SOCK_ID_TCP, Sn_MR_TCP, PORT_TCP, 0x00)) != SOCK_ID_TCP)
			{
				return;
			}

			break;
		}

		default:
		{
			break;
		}
	}
}


void TCP_Server(void)
{
	int32_t ret;
	uint16_t size = 0, sentsize = 0;
	int16_t funcVal  = 0;


	//uartPutsP("\nTCP_Server\n");
	// Get status of socket
	funcVal = getSn_SR(SOCK_ID_TCP);
	//sprintf(gPtrintBuff, "\ngetSn_SR(%i):  %#02x\n", SOCK_ID_TCP, funcVal);
	//uartPuts(gPtrintBuff);
	
	switch(funcVal){
		// Connection established
		case SOCK_ESTABLISHED :{
			//uartPutsP("\nScoket estabilished\n");
			// Check interrupt: connection with peer is successful
			if(getSn_IR(SOCK_ID_TCP) & Sn_IR_CON)
			{
				// Clear corresponding bit
				setSn_IR(SOCK_ID_TCP,Sn_IR_CON);
				//uartPutsP("\nConnection Successful\n");
			}

			// Get received data size
			if((size = getSn_RX_RSR(SOCK_ID_TCP)) > 0)
			{
				// Cut data to size of data buffer
				if(size > DATA_BUF_SIZE)
				{
					size = DATA_BUF_SIZE;
				}

				// Get received data
				ret = recv(SOCK_ID_TCP, gDATABUF, size);

				// Check for error
				if(ret <= 0)
				{
					return;
				}

				// Send echo to remote
				sentsize = 0;
				while(size != sentsize)
				{
					ret = send(SOCK_ID_TCP, gDATABUF + sentsize, size - sentsize);
					
					// Check if remote close socket
					if(ret < 0)
					{
						close(SOCK_ID_TCP);
						return;
					}

					// Update number of sent bytes
					sentsize += ret;
				}
			}
			break;
		}

		// Socket received the disconnect-request (FIN packet) from the connected peer
		case SOCK_CLOSE_WAIT :
		{
			// Disconnect socket
			if((ret = disconnect(SOCK_ID_TCP)) != SOCK_OK)
			{
				
				
				return;
			}

			break;
		}

		// Socket is opened with TCP mode
		case SOCK_INIT :{
			//uartPutsP("SOCK_INIT\n");
			// Listen to connection request
			//if( (ret = listen(SOCK_ID_TCP)) != SOCK_OK){

				
			//	return;
			//}

			break;
		}

		// Socket is released
		case SOCK_CLOSED:
		{
			// Open TCP socket
			if((ret = socket(SOCK_ID_TCP, Sn_MR_TCP, PORT_TCP, 0x00)) != SOCK_ID_TCP)
			{
				return;
			}

			break;
		}

		default:
		{
			break;
		}
	}
}

// brief Handle UDP socket state.
void UDP_Server(void)
{
	int32_t  ret;
	uint16_t size, sentsize;
	uint8_t  destip[4];
	uint16_t destport;

	// Get status of socket
	switch(getSn_SR(SOCK_ID_UDP))
	{
		// Socket is opened in UDP mode
		case SOCK_UDP:
		{
			// Get received data size
			if((size = getSn_RX_RSR(SOCK_ID_UDP)) > 0)
			{
				// Cut data to size of data buffer
				if(size > DATA_BUF_SIZE)
				{
					size = DATA_BUF_SIZE;
				}

				// Get received data
				ret = recvfrom(SOCK_ID_UDP, gDATABUF, size, destip, (uint16_t*)&destport);

				// Check for error
				if(ret <= 0)
				{
					return;
				}

				// Send echo to remote
				size = (uint16_t) ret;
				sentsize = 0;
				while(sentsize != size)
				{
					ret = sendto(SOCK_ID_UDP, gDATABUF + sentsize, size - sentsize, destip, destport);
					if(ret < 0)
					{
						return;
					}
					// Update number of sent bytes
					sentsize += ret;
				}
			}
			break;
		}

		// Socket is not opened
		case SOCK_CLOSED:
		{
			// Open UDP socket
			if((ret=socket(SOCK_ID_UDP, Sn_MR_UDP, PORT_UDP, 0x00)) != SOCK_ID_UDP)
			{
				return;
			}

			break;
		}

		default :
		{
			break;
		}
	}
}
*/
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

/*
void ip_init(void){
	wiz_NetInfo netInfo  = {
		{0x00, 0x14, 0xA3, 0x72, 0x17, 0x3f},    // Source Mac Address
		{10, 101,  14, 53},                      // Source IP Address
		{255, 255, 254, 0},                      // Subnet Mask
		{10, 101,  14, 1},                       // Gateway IP Address
		{10, 101,  14, 99},                      // DNS server IP Address
		NETINFO_STATIC
	};				// Gateway address
	uint8_t  bufSize[] = {2, 2, 2, 2};
	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
	wizphy_reset();
	_delay_ms(5);
	wizchip_init(bufSize, bufSize);
	//memcpy_P(&netInfo, &network_info, sizeof(wiz_NetInfo));
	wizchip_setnetinfo(&netInfo);
	sock_state=IP_STATE_NO_SOCKET;
}

void ip_task(void){
	
	int8_t errNum = 0;
	
	switch(sock_state){
		
		case IP_STATE_NO_SOCKET:
		//uartPutsP("\nState: IP_STATE NO SOCKET\n");
		
		errNum = socket(0, Sn_MR_TCP, 5000, 0);
		
		if(errNum == SOCKERR_SOCKNUM) {
			uartPutsP("\nERR: Invalid socket number!\n");
		}
		if(errNum == SOCKERR_SOCKMODE) {
			uartPutsP("\nERR: Not support socket mode as TCP, UDP, and so on\n");
		}
		if(errNum == SOCKERR_SOCKFLAG) {
			uartPutsP("\nERR: Invalid socket flag\n");
		}
		
		if( errNum == 0){
			sock_state = IP_STATE_SOCKET;
		}
		break;
		case IP_STATE_SOCKET:
		uartPutsP("\nState: IP_STATE SOCKET\n");
		switch (listen(0)){
			case SOCK_OK :
			uartPutsP("\nState:SOCK OK\n");
			sock_state = IP_STATE_LISTEN;
			break;
			case SOCKERR_SOCKCLOSED :
			uartPutsP("\nState:SOCK CLOSED\n");
			sock_state = IP_STATE_CLOSE;
		}
		break;
		case IP_STATE_LISTEN :
		uartPutsP("-");
		uint8_t getSN_response = getSn_SR(0);
		sprintf(gPtrintBuff, "getSn_SR(0): %i\n", getSN_response);
		switch(getSN_response){
			case SOCK_LISTEN :
			_delay_ms(100);
			break;
			case SOCK_ESTABLISHED:
			uartPutsP("\nState: IP_STAbilished\n");
			sock_state = IP_STATE_ECHO;
			break;
			default:
			sock_state = IP_STATE_CLOSE;
		}
		break;
		case IP_STATE_CLOSE:
		//uartPutsP("\nState: IP_STATE CLOSE\n");
		disconnect(0);
		close(0);
		sock_state=IP_STATE_NO_SOCKET;
		break;
		case IP_STATE_ECHO:
		uartPutsP("\nState: IP_STATE ECHO\n");
		if (getSn_SR(0) == SOCK_ESTABLISHED) {
			sock_state = echo();
			} else {
			sock_state = IP_STATE_CLOSE;
		}
		break;
		case IP_STATE_GREETING:
		//uartPutsP("\nState: IP_STATE GREETINGS\n");
		if (getSn_SR(0) == SOCK_ESTABLISHED) {
			sock_state = greeting();
			} else {
			sock_state = IP_STATE_CLOSE;
		}
		break;
	}
}

uint8_t echo(void){
	uartPutsP("\nECHO\n");
	int16_t size = getSn_RX_RSR(0);
	if (size == 0) return IP_STATE_ECHO;
	if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
	int32_t ret = recv(0,read_buffer,size);
	if (ret<=0)
	return IP_STATE_CLOSE;
	for (uint8_t i = 0 ; i < ret; i++){
		write_buffer[i]=toupper(read_buffer[i]);
	}
	int32_t send_ret = send(0, write_buffer, ret);
	if (ret != send_ret){
		return IP_STATE_CLOSE;
	}
	
	for (int i=0; i< ret ; i++){
		if (read_buffer[i]=='.'){
			return IP_STATE_GREETING;
		}
	}
	return IP_STATE_ECHO;
}

uint8_t greeting(void){
	send(0, GREETING_MSG, strlen(GREETING_MSG));
	return IP_STATE_CLOSE;
}

*/

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