/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

/* GPIO Connections of Character LCD
 * Character LCD is connected to PORTD and PB[2:0]
 */

#define DB0		IOPORT_CREATE_PIN(PORTD, 0)
#define DB1		IOPORT_CREATE_PIN(PORTD, 1)
#define DB2		IOPORT_CREATE_PIN(PORTD, 2)
#define DB3		IOPORT_CREATE_PIN(PORTD, 3)
#define DB4		IOPORT_CREATE_PIN(PORTD, 4)
#define DB5		IOPORT_CREATE_PIN(PORTD, 5)
#define DB6		IOPORT_CREATE_PIN(PORTD, 6)
#define DB7		IOPORT_CREATE_PIN(PORTD, 7)
#define DB		IOPORT_PORTD
#define RS		IOPORT_CREATE_PIN(PORTB, 0)
#define RW		IOPORT_CREATE_PIN(PORTB, 1)
#define E		IOPORT_CREATE_PIN(PORTB, 2)


/* GPIO Connections of I2S Bus
 * I2S Bus is connected to PORTC[5:4] and used as TWI function Port. 
 * Sub CPU operate as I2S slave. 
 */

#define SDA			IOPORT_CREATE_PIN(PORTC, 4)
#define SCL			IOPORT_CREATE_PIN(PORTC, 5)



#endif // USER_BOARD_H
