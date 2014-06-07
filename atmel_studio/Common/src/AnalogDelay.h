/*
 * AnalogDelay_v2.h
 *
 * Created: 2014/01/25 16:42:39
 *  Author: A.S
 */ 


#ifndef ANALOGDELAY_H_
#define ANALOGDELAY_H_

#define VOL_ADR_BYTE		AD5280_5282_ADR_BYTE
#define VOL_SUBADR_BIT		AD5280_5282_SUBADR_BIT
#define VOL_RS_BIT			AD5280_5282_RS_BIT
#define VOL_SD_BIT			AD5280_5282_SD_BIT
#define VOL_O1_BIT			AD5280_5282_O1_BIT
#define VOL_O2_BIT			AD5280_5282_O2_BIT

#define FBVOL_ADR_BYTE		AD5263_ADR_BYTE
#define FBVOL_ADR1_BIT		AD5263_ADR1_BIT
#define FBVOL_ADR0_BIT		AD5263_ADR0_BIT
#define FBVOL_RS_BIT		AD5263_RS_BIT
#define FBVOL_SD_BIT		AD5263_SD_BIT
#define FBVOL_O1_BIT		AD5263_O1_BIT
#define FBVOL_O2_BIT		AD5263_O2_BIT

#define LCD_ADDR_BYTE			0x20
#define LCD_CMD_LOCATE			0x01
#define LCD_CMD_PUTC			0x02
#define LCD_CMD_TRANSFER_REG	0x03
#define LCD_CMD_DISPLAY_VOLUME	0x04
#define LCD_CMD_DISPLAY_LOAD	0x05
#define LCD_CMD_DISPLAY_SAVE	0x06
#define LCD_CMD_PUTSTR			0x07

#define MESSAGEBUF_SIZE	10

#define ENCODER_INCREMENT	1
#define ENCODER_DECREMENT	0
#define ENCODER_ERROR		-1

#define DISPLAY_DELAY_TIME	0
#define DISPLAY_TEMPO		1

#define ON_OFF_BIT			0
#define INDET0_BIT			1
#define DRY_MIX_BIT			2
#define INDET1_BIT			3
#define OUTDET1_BIT			4
#define OUTDET0_BIT			5
#define DTYPE0_BIT			6
#define DTYPE1_BIT			7

#define get_bit_mask(bit)	0x01<<bit
#define set_bit_level_high(r, bit)	r |= 1<<bit
#define set_bit_level_low(r, bit)	r &= ~(1<<bit)
#define set_byte_level_high(r, mask) 	r |= mask
#define set_byte_level_low(r, mask) 	r &= ~mask

#define TEMPO_TIME_BIT		0
#define MIXSEL1_BIT			1
#define MIXSEL0_BIT			2
#define BSN_BIT				3
#define MBANK0_BIT			4
#define MBANK1_BIT			5
#define MBANK2_BIT			6

#define MAX_DELAY_TIME		999	// Maximum Delay Time(msec)
#define MIN_DELAY_TIME		40		// Minimum Delay Time(msec)
#define MAX_DELAY_TEMPO		500	// Maximum Delay Tempo
#define MIN_DELAY_TEMPO		60		// Minimum Delay Tempo

#endif /* ANALOGDELAY_V2_H_ */