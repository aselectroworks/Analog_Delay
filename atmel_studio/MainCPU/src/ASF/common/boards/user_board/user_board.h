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

/* GPIO Connections
 * 
 */

#define PORT_DET	IOPORT_PORTA
#define INDET0		IOPORT_CREATE_PIN(PORTA, 1)
#define INDET1		IOPORT_CREATE_PIN(PORTA, 3)
#define OUTDET0		IOPORT_CREATE_PIN(PORTA, 4)
#define OUTDET1		IOPORT_CREATE_PIN(PORTA, 5)
#define SW0			IOPORT_CREATE_PIN(PORTA, 7)
#define SW1			IOPORT_CREATE_PIN(PORTA, 6)

#define CLK_N		IOPORT_CREATE_PIN(PORTB, 0)
#define CLK_P		IOPORT_CREATE_PIN(PORTB, 1)
#define PORT_ENC	IOPORT_PORTB
#define ENC_A		IOPORT_CREATE_PIN(PORTB, 2)
#define ENC_B		IOPORT_CREATE_PIN(PORTB, 3)
#define VOL			IOPORT_CREATE_PIN(PORTB, 4)
#define FBVOL		IOPORT_CREATE_PIN(PORTB, 5)
#define ON_OFF		IOPORT_CREATE_PIN(PORTB, 6)

#define ADC_VOL		ADC_MUX_ADC7
#define ADC_FBVOL	ADC_MUX_ADC8


/* GPIO Connections of I2S Bus
 * I2C Bus is connected to PORTA[2]/PORTA[0] and used as TWI function Port using USI. 
 * Sub CPU operate as I2S slave. 
 */
#define I2C			&USICR
#define SDA			IOPORT_CREATE_PIN(PORTA, 0)
#define SCL			IOPORT_CREATE_PIN(PORTA, 2)

#endif // USER_BOARD_H
