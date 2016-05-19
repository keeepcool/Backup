#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define F_CPU	16000000UL
#define BAUD	57600
#include <util/setbaud.h>
#include "uart.h"

//Global variables declaration

static volatile unsigned char txBuff[TX_BUFFER_SIZE];
static volatile unsigned char rxBuff[RX_BUFFER_SIZE];
static volatile unsigned char txHead;
static volatile unsigned char txTail;
static volatile unsigned char rxHead;
static volatile unsigned char rxTail;
static volatile unsigned char lastRxError;

ISR(USART1_RX_vect){

	unsigned char tmpHead;
	unsigned char data;
	unsigned char usr;
	unsigned char rxError;

	usr = UCSR1A;
	data = UDR1;

	rxError = (usr & ((1<<FE1)|(1<<DOR1)));

	tmpHead = ((rxHead + 1) & RX_BUFFER_MASK);

	if(tmpHead == rxTail){
		rxError = UART_BUFFER_OVERFLOW >> 8;
	}
	else {
		rxHead = tmpHead;
		rxBuff[tmpHead] = data;
	}

	lastRxError = rxError;
}

ISR(USART1_UDRE_vect){

	unsigned char tmpTail;

	if(txHead != txTail){
		tmpTail = ((txTail +1) & TX_BUFFER_MASK);
		txTail = tmpTail;
		UDR1 = txBuff[tmpTail];
	}
	else {
		UCSR1B &= ~(1<<UDRIE1);
	}
}


void uartInit(void){

	//Init all the tails and heads to 0
	
	txHead = 0;
	txTail = 0;
	rxHead = 0;
	rxTail = 0;

	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
	#if USE_2X
	UCSR1A |= (1<<U2X1);
	#else
	UCSR1A &= ~(1<<U2X1);
	#endif

	UCSR1C |= ((1<<UCSZ11)|(1<<UCSZ10));
	UCSR1B |= ((1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1));
}

unsigned int uartGetc(void){

	unsigned char tmpTail;
	unsigned char data;

	if(rxHead == rxTail){
		return UART_NO_DATA;
	}

	tmpTail = ((rxTail + 1) & RX_BUFFER_MASK);
	rxTail = tmpTail;

	data = rxBuff[tmpTail];

	return ((lastRxError << 8) + data);
}

void uartPutc(unsigned char data){
	
	unsigned char tmpHead;

	tmpHead = ((txHead + 1) & TX_BUFFER_MASK);

	while(tmpHead == txTail){
		;
	}

	txBuff[tmpHead] = data;
	txHead = tmpHead;

	UCSR1B |= (1<<UDRIE1);
}

void uartPuts(const char *s){

	while(*s){
		uartPutc(*s++);
	}
}

void uart_puts_t(const char *s){

	register char c;

	while( (c = pgm_read_byte(s++))){
		uartPutc(c);
	}
}
