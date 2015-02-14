/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <stdlib.h>
#include <string.h>
#include "osslib/hd44780.h"
#include "osslib/xprintf.h"
#include "osslib/TWI_Slave.h"
#include "AnalogDelay.h"


// When there has been an error, this function is run and takes care of it
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );

void lcd_putstr(uint8_t *str) {

	uint8_t i = 0;

	while(*(str+i) != 0x00) {
		lcd_putc(*(str+i));
		i++;
	}

}

void set_custom_character() {
	// Custom Character Example
	uint8_t quarter_note_chr[8] = {0,4,4,4,4,28,28,0};				// Quarter note
	uint8_t dotted_quarter_note_chr[8] = {0,4,4,4,4,28,29,0};		// Dotted Quarter note
	uint8_t eighth_note_chr[8] = {0,6,7,5,4,28,28,0};				// Eighth note
	uint8_t dotted_eighth_note_chr[8] = {0,6,7,5,4,28,29,0};		// Dotted Eighth note
	uint8_t dotted_demiquaver_note_chr[8] = {12,14,11,12,14,11,24,25};		// Dotted demiquaver note
	uint8_t triplet_note_chr[8] = {8,8,8,11,25,27,1,3};				// Triplet
	uint8_t triplet_eighth_note_chr[8] = {12,14,8,11,25,27,1,3};				// Triplet eighth
					
	lcd_setcg(0x00, 1, (uint8_t*)quarter_note_chr);
	lcd_setcg(0x01, 1, (uint8_t*)dotted_eighth_note_chr);
	lcd_setcg(0x02, 1, (uint8_t*)eighth_note_chr);
	lcd_setcg(0x03, 1, (uint8_t*)dotted_demiquaver_note_chr);
	lcd_setcg(0x04, 1, (uint8_t*)triplet_note_chr);
	lcd_setcg(0x05, 1, (uint8_t*)triplet_eighth_note_chr);
	
}

void display_save(uint8_t DCR1) {
	
	irqflags_t flags;
	char STR[10] = {};
	uint8_t i;
	
	flags = cpu_irq_save();
	lcd_locate(0, 0);
	lcd_putstr("SAVE    ");
	lcd_locate(1, 0);
	lcd_putstr("   ");
	lcd_putc(0x7E);
	lcd_putstr("MEM");
	i = (DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))>>MBANK0_BIT;
	itoa(i, STR, 10);
	lcd_putstr(STR);

	cpu_irq_restore(flags);
	
}

void display_load(uint8_t DCR1) {
	
	irqflags_t flags;
	char STR[10] = {};
	uint8_t i, j;
	
	flags = cpu_irq_save();
	lcd_locate(0, 0);
	lcd_putstr("LOAD    ");
	lcd_locate(1, 0);
	lcd_putstr("   ");
	lcd_putc(0x7F);
	lcd_putstr("MEM");
	i = (DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))>>MBANK0_BIT;
	itoa(i, STR, 10);
	lcd_putstr(STR);

	cpu_irq_restore(flags);
	
}

void display_volume(uint8_t VOLR, uint8_t FBVR) {
	
	irqflags_t flags;
	char STR[10] = {};
	uint8_t i, j;
	
	flags = cpu_irq_save();
	lcd_locate(0, 0);
	lcd_putstr("VOL: ");
	itoa(VOLR, STR, 10);
	j = strlen(STR);
	for(i=3; i>j; i--) {
		lcd_putc(0x20);
	}
	lcd_putstr(STR);
	
	lcd_putstr("FB : ");
	itoa(FBVR, STR, 10);
	j = strlen(STR);
	for(i=3; i>j; i--) {
		lcd_putc(0x20);
	}
	lcd_putstr(STR);

	cpu_irq_restore(flags);
	
}
	
void display_status(uint8_t DCR0, uint8_t DCR1, uint16_t DTR, uint16_t TPR, uint8_t DCR2) {
	
	irqflags_t flags;
	char DTR_STR[10] = {};
	char TPR_STR[10] = {};
	uint8_t i, j;
	
	flags = cpu_irq_save();
	
	lcd_locate(0,0); 
	if(DCR0 & get_bit_mask(ON_OFF_BIT)) {
		if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT))) \
		== (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT))) {
			lcd_putstr("S.DELAY");
		}
		else {
			lcd_putstr("DELAY  ");
		}
	}
	else if(DCR0 & get_bit_mask(INDET1_BIT)) {
		if(((DCR0 & (get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) == (get_bit_mask(OUTDET1_BIT))) \
		|| ((DCR0 & (get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) == (get_bit_mask(OUTDET0_BIT)))) {
			lcd_putstr("BYPASS ");
		}
		else if((DCR0 & (get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(DRY_MIX_BIT))) \
		== ((get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT)))) {
			lcd_putstr("BYPASS ");
		}
		else {
			lcd_putstr("FXOFF  ");
		}
	} 
	else {
		lcd_putstr("FXOFF  ");
	}
	lcd_locate(0, 7);
	switch(DCR2&0x07) {		// Tempo mode Display switch
		case 0:
			lcd_putc(0x00);
		break;
		case 1:
			lcd_putc(0x01);
		break;
		case 2:
			lcd_putc(0x02);
		break;
		case 3:
			lcd_putc(0x03);
		break;
		case 4: 
			lcd_putc(0x04);
		break; 
		case 5:
			lcd_putc(0x05);
		break;
		default:
		break;
	}
	lcd_locate(1, 0);
	if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT))) && (DCR0 & (get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT)))) {
		if(DCR1 & get_bit_mask(BSN_BIT)) {
			if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
			get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT))) \
			== (get_bit_mask(INDET1_BIT) | get_bit_mask(OUTDET0_BIT))) {
				lcd_putstr(" ");
			}
			else {
				lcd_putstr("L");
			}
		}
		else {
			if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
			get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT))) \
			== (get_bit_mask(INDET1_BIT) | get_bit_mask(OUTDET1_BIT))) {
				lcd_putstr(" ");
			}
			else if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
			get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT) | get_bit_mask(DRY_MIX_BIT))) \
			== (get_bit_mask(INDET1_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) {
				lcd_putstr(" ");
			}
			else if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
			get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(DRY_MIX_BIT))) \
			== (get_bit_mask(INDET0_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) {
				lcd_putstr("S");
			}
			else if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
			get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT) |get_bit_mask(DRY_MIX_BIT))) \
			== (get_bit_mask(INDET1_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT))) {
				lcd_putstr("S");
			}
			else {
				lcd_putstr("S");
			}
		}
	}
	else {
		lcd_putstr(" ");
	}
	if((DCR0 & (get_bit_mask(DRY_MIX_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) \
	== (get_bit_mask(DRY_MIX_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) {
			lcd_putstr("D");
	}
	else if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
	get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(DRY_MIX_BIT))) \
	== (get_bit_mask(INDET0_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT))) {
		lcd_putstr("P");
	}
	else if((DCR0 & (get_bit_mask(INDET1_BIT) | get_bit_mask(INDET0_BIT) | \
	get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT) |get_bit_mask(DRY_MIX_BIT))) \
	== (get_bit_mask(INDET1_BIT) | get_bit_mask(OUTDET1_BIT) | get_bit_mask(OUTDET0_BIT) | get_bit_mask(ON_OFF_BIT))) {
		lcd_putstr("P");
	}
	else {
		//lcd_putstr("MIX ");
		lcd_putstr(" ");
	}
	switch(DCR1 & 0x01) {
		case DISPLAY_DELAY_TIME:
			itoa(DTR, DTR_STR, 10);
			j = strlen(DTR_STR);
			for(i=4; i>j; i--) {
				lcd_putc(0x20);
			}
			lcd_putstr(DTR_STR);
			lcd_putstr("ms"); 
		break;
		case DISPLAY_TEMPO:
			lcd_putstr(" ");
			lcd_putc(0x00);
			lcd_putstr("=");
			itoa(TPR, TPR_STR, 10);
			j = strlen(TPR_STR);
			for(i=3; i>j; i--) {
				lcd_putc(0x20);
			}
			lcd_putstr(TPR_STR);
		break;
		default:
		break;
	}
//	lcd_locate(1, 0);
//	itoa(DCR0, TPR_STR, 16);
//	lcd_putstr(TPR_STR);
//	itoa(DCR1, TPR_STR, 16);
//	lcd_putstr(TPR_STR);
	
	cpu_irq_restore(flags);
}
	
int main (void)
{
	unsigned char messageBuf[TWI_BUFFER_SIZE] = {};
	unsigned char TWI_slaveAddress = LCD_ADDR_BYTE;	
	uint16_t i = 0;
	uint8_t DCR0 = 0;
	uint8_t DCR1 = 0;
	uint16_t DTR = 0;
	uint16_t TPR = 0;
	uint8_t VOLR = 80;
	uint8_t FBVR = 55;
	uint8_t DCR2 = 0; 
	board_init();
	// Insert application code here, after the board has been initialized.
	//delay_ms(100);
	// Initialise LCD device
	lcd_init();
	set_custom_character();
	xdev_out(lcd_putc);
	// Initialise TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) ));
	cpu_irq_enable();
	// Start the TWI transceiver to enable reseption of the first command from the TWI Master.
	TWI_Start_Transceiver();
	

	for(i=10; i<1000; i++) {
	if(i==89) i=10;
		if ( ! TWI_Transceiver_Busy() )
		{
			// Check if the last operation was successful
			if ( TWI_statusReg.lastTransOK )
			{
				// Check if the last operation was a reception
				if ( TWI_statusReg.RxDataInBuf )
				{
					TWI_Get_Data_From_Transceiver(messageBuf, 10);
					// Check if the last operation was a reception as General Call
					if ( TWI_statusReg.genAddressCall )
					{
						// Put data received out to PORTB as an example.
						//PORTB = messageBuf[0];
					}
					else // Ends up here if the last operation was a reception as Slave Address Match
					{
						// Example of how to interpret a command and respond.
						
						// TWI_CMD_MASTER_WRITE stores the data to PORTB
						switch(messageBuf[0]){
							case LCD_CMD_LOCATE: 
								lcd_locate(messageBuf[1]>>4, messageBuf[1] & 0x0F);
							break;
							case LCD_CMD_PUTC:
								lcd_putc(messageBuf[1]);
							break;
							case LCD_CMD_TRANSFER_REG: 
								DCR0 = messageBuf[1];
								DCR1 = messageBuf[2];
								DTR = (messageBuf[3] << 8 | messageBuf[4]);
								TPR = (messageBuf[5] << 8 | messageBuf[6]);
								VOLR = messageBuf[7];
								FBVR = messageBuf[8];
								DCR2 = messageBuf[9];
								display_status(DCR0, DCR1, DTR, TPR, DCR2);
							break;
							case LCD_CMD_DISPLAY_VOLUME:
								DCR0 = messageBuf[1];
								DCR1 = messageBuf[2];
								DTR = (messageBuf[3] << 8 | messageBuf[4]);
								TPR = (messageBuf[5] << 8 | messageBuf[6]);
								VOLR = messageBuf[7];
								FBVR = messageBuf[8];
								DCR2 = messageBuf[9];
								display_volume(VOLR, FBVR);
							break;
							case LCD_CMD_DISPLAY_SAVE:
								DCR0 = messageBuf[1];
								DCR1 = messageBuf[2];
								DTR = (messageBuf[3] << 8 | messageBuf[4]);
								TPR = (messageBuf[5] << 8 | messageBuf[6]);
								VOLR = messageBuf[7];
								FBVR = messageBuf[8];
								DCR2 = messageBuf[9];
								display_save(DCR1);
							break;
							case LCD_CMD_DISPLAY_LOAD:
								DCR0 = messageBuf[1];
								DCR1 = messageBuf[2];
								DTR = (messageBuf[3] << 8 | messageBuf[4]);
								TPR = (messageBuf[5] << 8 | messageBuf[6]);
								VOLR = messageBuf[7];
								FBVR = messageBuf[8];
								DCR2 = messageBuf[9];
								display_load(DCR1);
							break;
							default:
							break;
						}
					}				}
				else // Ends up here if the last operation was a transmission
				{
					 nop(); // Put own code here.
				}
				// Check if the TWI Transceiver has already been started.
				// If not then restart it to prepare it for new receptions.
				if ( ! TWI_Transceiver_Busy() )
				{
					TWI_Start_Transceiver();
				}
			}
			else // Ends up here if the last operation completed unsuccessfully
			{
				TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info() );

			}
		}

    }
}


unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	
	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
	
	//PORTB = TWIerrorMsg;

	//int8_t i;
	//for (i=0; i<8; i++) {
	//	lcd_putc((((TWIerrorMsg & (1 << i))>>i) +0x30));
	//}
		
	TWI_Start_Transceiver();	
	return TWIerrorMsg;
}