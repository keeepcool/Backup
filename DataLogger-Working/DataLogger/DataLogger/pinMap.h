/*
 * pinMap.h
 *
 * Created: 20/01/2016 17:09:32
 *  Author: Tiago
 */ 


#ifndef PINMAP_H_
#include <avr/io.h>

#define PINMAP_H_

//PORT E PINS
#define LED_PIN		PORTE3		//OC3A
#define LED_DDR		DDRE
#define LED_PORT	PORTE

#define RX			PORTE0
#define TX			PORTE1
#define TRX_PIN		PINE
#define TRX_DDR		DDRE
#define TRX_PORT	PORTE

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

//PORT F PINS
#define AUX0_PIN	PORTF6		//ADC channel 6
#define AUX1_PIN	PORTF7		//ADC channel 7
#define AUXO1_DDR	DDRF
#define AUX01_PIN	PINF
#define AUX01_PORT	PORTF

//PORT B PINS
#define AUX2_PIN	PORTB7		//OC0A:PCINT7
#define AUX3_PIN	PORTB6		//OC1B:PCINT6
#define AUX23_DDR	DDRB	
#define AUX23_PIN	PINB
#define AUX23_PORT	PORTB

#define INT1ACC_PIN	PORTB5		//PCINT5

#define SCK			PORTB1
#define MOSI		PORTB2
#define	MISO		PORTB3

//PORT G PIN
#define PLEN_PIN	PORTG0
#define WP_PIN		PORTG1
#define CTRL_PORT	PORTG
#define CTRL_PIN	PING
#define CTRL_DDR	DDRG

#endif /* PINMAP_H_ */