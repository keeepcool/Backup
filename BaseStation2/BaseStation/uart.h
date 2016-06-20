#ifndef UART_H
#define UART_H

#define TX_BUFFER_SIZE	32
#define RX_BUFFER_SIZE	32
#define TX_BUFFER_MASK	(TX_BUFFER_SIZE-1)
#define RX_BUFFER_MASK	(RX_BUFFER_SIZE-1)

#define UART_FRAME_ERROR	0x0800
#define UART_OVERRUN_ERROR	0x0400
#define UART_BUFFER_OVERFLOW	0x0200
#define UART_NO_DATA		0x0100


/**
 * Does all the init process and sets the baud-rate definided in the begin of
 * the uart.c file
 */

void uartInit(void);

/**
 * Gets a received byte from the circular buffer
 * The lower byte is the received char and the higher
 * byte is the last receive error
 * Returns UART_NO_DATA when no data is available
 * High byte can be:
 * 0 is no error reported
 * UART_NO_DATA menas that the circular buffer is empty
 * UART_BUFFER_OVERFLOW means that the ciruclar buffer as overflowed
 * UART_OVERRUN_ERROR means that a byte was not read in tyme from the UDR register
 * UART_FRAME_ERROR means a framing error in the UART
 */
unsigned int uartGetc(void);


/**
 * Put a byte in the circular buffer to be transmitted by the UART
 */
void uartPutc(unsigned char data);

/**
 * Puts a string in the circular buffer to be transmitted
 * if the string fits in the actual buffer its all interrupt driven
 * if the string is bigger than the actual buffer it blocks
 */
void uartPuts(const char *s);

/**
 * Does the same as the above but uses data from the flash and not
 * from the RAM, use this with static strings to save ram!!!
 */

void uart_puts_t(const char *s);

/**
 * Check if there is new data to be read.
 */
uint16_t uartAvailable(void);


/**
 * This macro automagically puts the string/char into program memory
 */

#define uartPutsP(__s)	uart_puts_t(PSTR(__s))

#endif
