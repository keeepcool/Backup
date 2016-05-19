/*
 * pinMap.h
 *
 * Created: 21/04/2016 17:19:49
 *  Author: Tiago
 */ 


#ifndef PINMAP_H_
#define PINMAP_H_

//Global defines


/*
#define IP_STATE_NO_SOCKET	0
#define IP_STATE_SOCKET		1
#define IP_STATE_LISTEN     2
#define IP_STATE_CLOSE		3
#define IP_STATE_ECHO		4
#define IP_STATE_GREETING   5
#define IP_STATE_DATA		IP_STATE_ECHO
#define DATA_BUF_SIZE 80

const wiz_NetInfo PROGMEM network_info  = { .mac 	= {0x00, 0x08, 0xdc, 0xFF, 0xFF, 0xFF},	// Mac address: Please use your own MAC address.
.ip 	= {192, 168, 178, 201},					// IP address
.sn 	= {255, 255, 255, 0},					// Subnet mask
.gw 	= {192, 168, 178, 1}};					// Gateway address

*/

//TX RX 0 connected to BT module
//TX RX 1 connected to USB-Serial interface
//PORT D PINS
#define SCL			PORTD0
#define SDA			PORTD1
#define TWI_DDR		DDRD
#define TWI_PORT	PORTD

#define INT_RTC		PORTD2		//INT2
#define INT_ADPS	PORTD3		//INT3
#define INT_PORT	PORTD
#define INT_DDR		DDRD
#define INT_PIN		PIND

//Wiznet5500 reset and PMOD
#define RST_W5500	PD4
#define PMOD0		PD5
#define PMOD1		PD6
#define PMOD2		PD7
#define PMOD_DDR	DDRD
#define PMOD_PIN	PIND
#define PMOD_PORT	PORTD
//***********************************

//PORT E PINS
#define SS_W5500	PORTE2
#define INT_W5500	PORTE4

#define WIZ_DDR		DDRE
#define WIZ_PORT	PORTE
#define WIZ_PIN		PINE
//***********************************

//PORT F PINS
#define	BTN0		PORTF2
#define ISENSE		PORTF0		//ADC channel 0
#define VSENSE		PORTF1		//ADC channel 1
#define WP_EEP		PORTF3
#define AUX_DDR		DDRF
#define AUX_PIN		PINF
#define AUX_PORT	PORTF
//***********************************

//PORT B PINS

#define LED_R		PORTB5
#define LED_G		PORTB6
#define LED_B		PORTB7

#define LED_DDR		DDRB

#define SCK			PORTB1
#define MOSI		PORTB2
#define	MISO		PORTB3
//***********************************

//PORT G PIN




#endif /* PINMAP_H_ */