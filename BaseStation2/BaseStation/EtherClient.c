/*
 * EtherClient.c
 *
 * Created: 13/05/2016 15:17:24
 *  Author: Tiago
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
#include "socket.h"
#include "w5500.h"
#include "uart.h"

/*

static socket = 0;

int EtherConnect(*ip, uint16_t port) {
	if (sock != MAX_SOCK_NUM)
	return 0;

	for (int i = 0; i < MAX_SOCK_NUM; i++) {
		uint8_t s = W5100.readSnSR(i);
		if (s == SnSR::CLOSED || s == SnSR::FIN_WAIT || s == SnSR::CLOSE_WAIT) {
			_sock = i;
			break;
		}
	}

	if (_sock == MAX_SOCK_NUM)
	return 0;

	_srcport++;
	if (_srcport == 0) _srcport = 1024;
	socket(_sock, SnMR::TCP, _srcport, 0);

	if (!::connect(_sock, rawIPAddress(ip), port)) {
		_sock = MAX_SOCK_NUM;
		return 0;
	}

	while (status() != SnSR::ESTABLISHED) {
		delay(1);
		if (status() == SnSR::CLOSED) {
			_sock = MAX_SOCK_NUM;
			return 0;
		}
	}

	return 1;
}

size_t EthernetClient::write(uint8_t b) {
	return write(&b, 1);
}

size_t EthernetClient::write(const uint8_t *buf, size_t size) {
	if (_sock == MAX_SOCK_NUM) {
		setWriteError();
		return 0;
	}
	if (!send(_sock, buf, size)) {
		setWriteError();
		return 0;
	}
	return size;
}

int EthernetClient::available() {
	if (_sock != MAX_SOCK_NUM)
	return W5100.getRXReceivedSize(_sock);
	return 0;
}

int EthernetClient::read() {
	uint8_t b;
	if ( recv(_sock, &b, 1) > 0 )
	{
		// recv worked
		return b;
	}
	else
	{
		// No data available
		return -1;
	}
}

int EthernetClient::read(uint8_t *buf, size_t size) {
	return recv(_sock, buf, size);
}

*/