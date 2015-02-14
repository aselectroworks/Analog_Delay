/*
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
#include "osslib/USI_TWI_Master.h"
#include "AnalogDelay.h"
#include "AD5280_5282.h"
#include "AD5263.h"
#include <string.h>

// Prototype Declaration
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char );
uint16_t Get_Tempo_to_Delay_Time_ms(uint16_t , uint8_t );
uint16_t Get_Delay_Time_ms_to_Tempo(uint16_t , uint8_t );
void EEPROM_write(unsigned char , unsigned char );
unsigned char EEPROM_read(unsigned char );
void timer0_init(void);
void timer1_init(void);
uint8_t sample_encoder(void);

	
//#define __DELAY_CYCLE_INTRINSICS__
PROGMEM_DECLARE(uint8_t, adc_to_wiper[1024]) = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
	4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,9,
	9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,12,
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	13,13,13,13,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,
	17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,19,19,
	19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,21,
	21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,24,24,24,24,24,24,24,24,24,24,24,24,24,24,25,25,25,25,25,25,
	25,25,25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,27,27,
	27,27,27,28,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,29,29,30,30,30,30,
	30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,
	33,33,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,35,
	36,36,36,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,39,39,
	39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,40,40,41,41,41,41,41,41,41,41,41,42,42,42,42,42,42,
	42,42,42,43,43,43,43,43,43,43,43,44,44,44,44,44,44,44,44,44,45,45,45,45,45,45,45,45,46,46,46,46,
	46,46,46,46,47,47,47,47,47,47,47,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,50,50,50,50,50,50,
	50,51,51,51,51,51,51,51,52,52,52,52,52,52,52,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,55,
	55,56,56,56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,58,59,59,59,59,59,60,60,60,60,60,60,61,61,
	61,61,61,62,62,62,62,62,63,63,63,63,63,64,64,64,64,64,65,65,65,65,66,66,66,66,66,67,67,67,67,68,
	68,68,68,68,69,69,69,69,70,70,70,70,71,71,71,71,72,72,72,72,73,73,73,73,74,74,74,75,75,75,75,76,
	76,76,77,77,77,77,78,78,78,79,79,79,80,80,80,81,81,81,82,82,82,83,83,83,84,84,84,85,85,85,86,86,
	87,87,87,88,88,89,89,89,90,90,91,91,92,92,92,93,93,94,94,95,95,96,96,97,97,98,98,99,99,100,101,101,
	102,102,103,103,104,105,105,106,107,107,108,109,109,110,111,112,112,113,114,115,116,117,117,118,119,120,121,122,123,124,125,127,
	128,129,130,131,133,134,136,137,139,140,142,144,146,148,150,152,154,157,160,162,166,169,173,177,182,188,195,203,213,228,254,255
};

/*PROGMEM_DECLARE(uint8_t, adc_to_wiper[1024]) = {
0,0,6,22,34,43,50,56,61,66,70,74,78,81,84,87,
89,92,94,96,98,100,102,104,105,107,109,110,112,113,114,116,
117,118,119,120,122,123,124,125,126,127,128,129,130,131,131,132,
133,134,135,136,136,137,138,139,139,140,141,141,142,143,143,144,
145,145,146,146,147,148,148,149,149,150,150,151,152,152,153,153,
154,154,155,155,156,156,156,157,157,158,158,159,159,160,160,160,
161,161,162,162,162,163,163,164,164,164,165,165,166,166,166,167,
167,167,168,168,168,169,169,169,170,170,170,171,171,171,172,172,
172,173,173,173,174,174,174,174,175,175,175,176,176,176,177,177,
177,177,178,178,178,178,179,179,179,179,180,180,180,181,181,181,
181,182,182,182,182,183,183,183,183,183,184,184,184,184,185,185,
185,185,186,186,186,186,186,187,187,187,187,188,188,188,188,188,
189,189,189,189,189,190,190,190,190,190,191,191,191,191,191,192,
192,192,192,192,193,193,193,193,193,193,194,194,194,194,194,195,
195,195,195,195,195,196,196,196,196,196,196,197,197,197,197,197,
198,198,198,198,198,198,198,199,199,199,199,199,199,200,200,200,
200,200,200,201,201,201,201,201,201,201,202,202,202,202,202,202,
203,203,203,203,203,203,203,204,204,204,204,204,204,204,205,205,
205,205,205,205,205,205,206,206,206,206,206,206,206,207,207,207,
207,207,207,207,207,208,208,208,208,208,208,208,209,209,209,209,
209,209,209,209,210,210,210,210,210,210,210,210,210,211,211,211,
211,211,211,211,211,212,212,212,212,212,212,212,212,212,213,213,
213,213,213,213,213,213,214,214,214,214,214,214,214,214,214,214,
215,215,215,215,215,215,215,215,215,216,216,216,216,216,216,216,
216,216,217,217,217,217,217,217,217,217,217,217,218,218,218,218,
218,218,218,218,218,218,219,219,219,219,219,219,219,219,219,219,
220,220,220,220,220,220,220,220,220,220,220,221,221,221,221,221,
221,221,221,221,221,221,222,222,222,222,222,222,222,222,222,222,
222,223,223,223,223,223,223,223,223,223,223,223,224,224,224,224,
224,224,224,224,224,224,224,224,225,225,225,225,225,225,225,225,
225,225,225,225,226,226,226,226,226,226,226,226,226,226,226,226,
227,227,227,227,227,227,227,227,227,227,227,227,227,228,228,228,
228,228,228,228,228,228,228,228,228,229,229,229,229,229,229,229,
229,229,229,229,229,229,229,230,230,230,230,230,230,230,230,230,
230,230,230,230,231,231,231,231,231,231,231,231,231,231,231,231,
231,231,232,232,232,232,232,232,232,232,232,232,232,232,232,232,
233,233,233,233,233,233,233,233,233,233,233,233,233,233,233,234,
234,234,234,234,234,234,234,234,234,234,234,234,234,234,235,235,
235,235,235,235,235,235,235,235,235,235,235,235,235,236,236,236,
236,236,236,236,236,236,236,236,236,236,236,236,236,237,237,237,
237,237,237,237,237,237,237,237,237,237,237,237,237,238,238,238,
238,238,238,238,238,238,238,238,238,238,238,238,238,239,239,239,
239,239,239,239,239,239,239,239,239,239,239,239,239,239,240,240,
240,240,240,240,240,240,240,240,240,240,240,240,240,240,240,240,
241,241,241,241,241,241,241,241,241,241,241,241,241,241,241,241,
241,242,242,242,242,242,242,242,242,242,242,242,242,242,242,242,
242,242,242,242,243,243,243,243,243,243,243,243,243,243,243,243,
243,243,243,243,243,243,244,244,244,244,244,244,244,244,244,244,
244,244,244,244,244,244,244,244,244,244,245,245,245,245,245,245,
245,245,245,245,245,245,245,245,245,245,245,245,245,246,246,246,
246,246,246,246,246,246,246,246,246,246,246,246,246,246,246,246,
246,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,
247,247,247,247,247,247,248,248,248,248,248,248,248,248,248,248,
248,248,248,248,248,248,248,248,248,248,248,249,249,249,249,249,
249,249,249,249,249,249,249,249,249,249,249,249,249,249,249,249,
249,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,
250,250,250,250,250,250,250,251,251,251,251,251,251,251,251,251,
251,251,251,251,251,251,251,251,251,251,251,251,251,251,252,252,
252,252,252,252,252,252,252,252,252,252,252,252,252,252,252,252,
252,252,252,252,252,252,253,253,253,253,253,253,253,253,253,253,
253,253,253,253,253,253,253,253,253,253,253,253,253,253,254,254,
254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};*/

void timer0_init() {
	OCR0A = 250;		// 16MHz/64/OCR0A=1kHz, OCR0A=250
	// Configure Timer/Counter to CTC mode, and set desired prescaler
	TCCR0A = (1 << WGM00);
	TCCR0B = (0 << CS02 | 1 << CS01 | 1 << CS00);
	// Enable interrupt for compare match
	TIMSK = 1 << OCIE0A;
}

void timer1_init() {
	//! PLL Start Setting
	TCCR1B = 1 << PSR1;
	PLLCSR = 1 << PLLE;
	delay_us(200);
	while(!(PLLCSR & (1<<PLOCK)));
	PLLCSR |= (1<<PCKE);
	
	//OCR1A = 488; 
	//OCR1C = 976; 
	//TCCR1B = 0b0100 << CS10;
	TCCR1A = (0 << COM1A1 | 1 << COM1A0 | 0 << COM1B1 | 1 << COM1B0 | 0 << FOC1B | 0 << FOC1A | 1 << PWM1A | 0 << PWM1B);
	TIMSK |= 0<<TOIE1;
	
	
}

uint8_t sample_encoder() {
	
	static uint8_t i;
	
	i = (i << 2) + (ioport_get_pin_level(ENC_B) << 1 | ioport_get_pin_level(ENC_A));
	i &= 15;
	
	switch (i) { 
	case 0b00000111: 
		return ENCODER_INCREMENT;
		break;
	case 0b00001101: 
		return ENCODER_DECREMENT;
		break;
	default:
		break;
	}
	return ENCODER_ERROR;
}

uint16_t Get_Delay_Time_ms_to_Tempo(uint16_t delay_time_ms, uint8_t delay_mode) {
	// delay_time_ms: delay time[msec]
	// delay_mode   : 0:Quarter note, 1: Dotted Eighth note, 2: Eighth note, 3: Dotted sixteenth note, 4: Triplet, 5: Triplet eighth
	// return       : Tempo
	uint16_t tempo = 0;
	
	switch(delay_mode) {
		case 0: 
			tempo = 60000UL/delay_time_ms;		// quarter note
			break;
		case 1: 
			tempo = 60000UL/delay_time_ms*3/4;	// Dotted eighth note
			break;
		case 2: 
			tempo = 60000UL/delay_time_ms*1/2;	// eighth note
			break;
		case 3: 
			tempo = 60000UL/delay_time_ms*3/8;	// Dotted sixteenth note
			break; 
		case 4:
			tempo = 60000UL/delay_time_ms*1/3;	// Triplet
			break;
		case 5: 
			tempo = 60000UL/delay_time_ms*1/6;	// Triplet eighth
			break; 
		default:
			break;
	}
	return tempo; 
}

uint16_t Get_Tempo_to_Delay_Time_ms(uint16_t tempo, uint8_t delay_mode) {
	// tempo
	// delay_mode   : 0:Quarter note, 1: Dotted quarter note, 2: Eighth note, 3: Dotted eighth note, 4: Triplet
	// return - delay_time_ms: delay time[msec]
	uint32_t delay_time_ms = 0;
	
	switch(delay_mode) {
		case 0:
			delay_time_ms = 60000/tempo;
			break;
		case 1:
			delay_time_ms = 60000/tempo*3/4;
			break;
		case 2:
			delay_time_ms = 60000/tempo*1/2;
			break;
		case 3: 
			delay_time_ms = 60000/tempo*3/8; 
			break; 
		case 4:
			delay_time_ms = 60000/tempo*1/3;
			break;
		case 5:
			delay_time_ms = 60000/tempo*1/6;
			break;
		default:
			break;
	}
	return (uint16_t)(delay_time_ms);
}
	
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	asm("nop");
	return TWIerrorMsg;
}

void EEPROM_write(unsigned char ucAddress, unsigned char ucData)
{
	irqflags_t flags;
	flags = cpu_irq_save();
	
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0<<EEPM0);
	/* Set up address and data registers */
	EEAR = ucAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
	
	cpu_irq_restore(flags);
}

unsigned char EEPROM_read(unsigned char ucAddress)
{
	irqflags_t flags;
	flags = cpu_irq_save();

	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address register */
	EEAR = ucAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	cpu_irq_restore(flags);
	/* Return data from data register */
	return EEDR;
}
int main (void)
{
	// Variable for TWI-I2C
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	unsigned char TWI_targetSlaveAddress, temp;
	// Variable for Analog Delay Registers
	uint8_t loop_counter = 0;		// Main Loop Counter
	uint8_t sample_switch0 = 0;		// SW0 sample record
	uint8_t sw0_push_count =0;		// SW0 push timer/counter per 10msec
	uint8_t sample_switch1 = 0;		// SW1 sample record
	uint16_t sw1_push_count =0;		// SW1 push timer/counter per 1msec;
	uint16_t sw1_tap_count =0x3FF;		// SW1 tap timer/counter per 1msec	
	uint8_t DCR0 = 0x00;
	uint8_t DCR1 = get_bit_mask(TEMPO_TIME_BIT);
	uint8_t DCR2 = 0x00;
	uint16_t TPR = 120;
	uint16_t DTR = Get_Tempo_to_Delay_Time_ms(TPR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));	// Transfer Gotten Delay Time to Tempo
	uint8_t VOLR = 0x00;
	uint8_t FXVOLUME[2] = {0};
	uint8_t FBVR = 0x00;
	uint8_t FBVOLUME[4] = {0};
	uint16_t OCR = 250;				// Initialize Pre-OCR Register
	uint8_t LCD_CMD = LCD_CMD_TRANSFER_REG;
	uint8_t i;
	uint8_t i2c_send_time_counter = 0; 
	unsigned char msg_save[] = "Saving  ";
	unsigned char msg_load[] = "Loading ";
		
	board_init();
	
	// Insert application code here, after the board has been initialized.
	USIPP = 0x01;
	USI_TWI_Master_Initialise();
	timer0_init();
	timer1_init();
	adc_init(ADC_PRESCALER_DIV32);
	//cpu_irq_enable();
	delay_ms(500);					// Wait for Voltage rise up
	
	// Main Loop Start
	while(1) {
		if(TIFR & (1 << OCF0A)) {	// Interrupt per 1msec by timer/counter0
			// Transfer Display Data
			if(loop_counter==5 || loop_counter==0) {
				// Set Display Data to LCD
				//I2C_Transfer(ADDR, CMD, DATA, databyte, R/W);
				TWI_targetSlaveAddress = LCD_ADDR_BYTE;
				messageBuf[0]  = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
				messageBuf[1]  = LCD_CMD;             // The first byte is used for commands.
				messageBuf[2]  = DCR0;
				messageBuf[3]  = DCR1;
				messageBuf[4]  = (DTR>>8);
				messageBuf[5]  = (DTR&0xFF);
				messageBuf[6]  = (TPR>>8);
				messageBuf[7]  = (TPR&0xFF);
				messageBuf[8]  = VOLR;
				messageBuf[9]  = FBVR;
				messageBuf[10] = DCR2;
				temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 11);
				if (!temp)    // One of the operations failed.
				{             // Use TWI status information to detemine cause of failure and take appropriate actions.
					TWI_Act_On_Failure_In_Last_Transmission( USI_TWI_Get_State_Info( ) );
					asm("nop"); // Put own code here.
				}
			}
	
			// Input/Output Jack Detection
			if(loop_counter==3) {
				DCR0 = (DCR0 & ~(0b00111010)) | (ioport_get_port_level(PORT_DET, 0b00111010));
			}
			
			// SW0 State Check
			if(loop_counter==4) {
				sample_switch0 =sample_switch0 << 1 | ioport_get_pin_level(SW0);
				if((sample_switch0 & 0x03)==0x00) {
					if(sw0_push_count!=0xFF) {
						sw0_push_count++;
						if((LCD_CMD!=LCD_CMD_DISPLAY_LOAD)&&(LCD_CMD!=LCD_CMD_DISPLAY_SAVE)) {
							if(sw0_push_count==100) {		// Count 1sec
								// Long push
								if(sw1_push_count>500) {	// Change Display Mode
									DCR1 = (DCR1 & ~(get_bit_mask(TEMPO_TIME_BIT))) | (~DCR1 & get_bit_mask(TEMPO_TIME_BIT));
									sw1_push_count = 1023;	// timeout sw1_push_count
								}
								else {
									switch(DCR0 & (get_bit_mask(INDET0_BIT)|get_bit_mask(INDET1_BIT)|get_bit_mask(OUTDET0_BIT)|get_bit_mask(OUTDET1_BIT))) {
										case (get_bit_mask(OUTDET0_BIT)|get_bit_mask(OUTDET1_BIT)):
											// Toggle MIX/DRY mode
											DCR0 = (DCR0 & ~(get_bit_mask(DRY_MIX_BIT))) | (~DCR0 & get_bit_mask(DRY_MIX_BIT));
										break;
										case (get_bit_mask(INDET0_BIT)|get_bit_mask(OUTDET0_BIT)|get_bit_mask(OUTDET1_BIT)): 
											// Toggle MIX/DRY mode
											DCR0 = (DCR0 & ~(get_bit_mask(DRY_MIX_BIT))) | (~DCR0 & get_bit_mask(DRY_MIX_BIT));
										break;
										case (get_bit_mask(INDET1_BIT)|get_bit_mask(OUTDET0_BIT)|get_bit_mask(OUTDET1_BIT)): 
											// Toggle MIX/DRY mode
											DCR0 = (DCR0 & ~(get_bit_mask(DRY_MIX_BIT))) | (~DCR0 & get_bit_mask(DRY_MIX_BIT));
										break;
										default: 
										break;
									}
								}
							}
						}
					}
				}
				else if((sample_switch0 & 0x0F)==0x01) {
					if((sw0_push_count > 0) && (sw0_push_count < 25) && (sw1_push_count == 0)) {
						// Short_push
						// Effect ON/OFF
						DCR0 = (DCR0 & ~(get_bit_mask(ON_OFF_BIT))) | (~DCR0 & get_bit_mask(ON_OFF_BIT));
						ioport_set_pin_level(ON_OFF, DCR0&get_bit_mask(ON_OFF_BIT));
					}
					sw0_push_count = 0;
				}
			}
			
			sample_switch1 = sample_switch1 << 1 | ioport_get_pin_level(SW1);
			// SW1 State Check for TAP(every 1ms Main Loop)
			if((sample_switch1 & 0x03)==0x02) {
				if((LCD_CMD!=LCD_CMD_DISPLAY_LOAD)&&(LCD_CMD!=LCD_CMD_DISPLAY_SAVE)) {
					if(sw1_tap_count!=0x3FF){
						sw1_tap_count = (uint16_t)Get_Delay_Time_ms_to_Tempo(sw1_tap_count, 0);
						if((sw1_tap_count>=MIN_DELAY_TEMPO) && (sw1_tap_count<=MAX_DELAY_TEMPO)) {
							TPR = (TPR + sw1_tap_count) >> 1;			// If Tap < 1023msec, Write Delay Time
							DTR = Get_Tempo_to_Delay_Time_ms(TPR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));	// Transfer Gotten Delay Time to Tempo
						}
					}
					sw1_tap_count = 0;					// Clear at pushing SW1
				}
				else {
					switch(LCD_CMD) {
						case LCD_CMD_DISPLAY_SAVE: 
							// Save Tempo
							EEPROM_write((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x00, (unsigned char)(DTR&0xFF));
							EEPROM_write((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x01, (unsigned char)(DTR>>8));
							EEPROM_write((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x02, (unsigned char)(TPR&0xFF));
							EEPROM_write((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x03, (unsigned char)(TPR>>8));
							EEPROM_write((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x04, (unsigned char)(DCR2));
						break;
						case LCD_CMD_DISPLAY_LOAD:
							// Load Tempo
							DTR = EEPROM_read((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x00);
							DTR = (DTR&0x00FF) | (EEPROM_read((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x01)<<8);
							TPR = EEPROM_read((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x02);
							TPR = (TPR&0x00FF) | (EEPROM_read((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x03)<<8);
							DCR2 = EEPROM_read((DCR1&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)))+0x04);
						break;
						default:
						break;
					}
				}
				
			}
			else {
				if(sw1_tap_count!=0x3FF) {
					sw1_tap_count++;				// Count up till 1023
				}
			}
			// SW1 State Check for PUSH(every 1ms Main Loop)
			if((LCD_CMD!=LCD_CMD_DISPLAY_LOAD)&&(LCD_CMD!=LCD_CMD_DISPLAY_SAVE)) {
				if((sample_switch1 & 0x03)==0x00) {
					if(sw1_push_count!=0x3FF) {
						sw1_push_count++;
						if(sw1_push_count==1000) {		// Count 1sec
							// Long push Detect
							// Toggle Delay mode
							if((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT == 0x05){
								DCR2 = (DCR2&~(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)));
							}
							else {
								DCR2++;
							}
							DTR = Get_Tempo_to_Delay_Time_ms(TPR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));
						}
					}
				}
				else if((sample_switch1 & 0x0F)==0x01) {
					sw1_push_count = 0;
				}				
			}
			
			// Volume Pot ADC
			if(loop_counter==6) {
				VOLR = PROGMEM_READ_BYTE(&(adc_to_wiper[adc_read_10bit(ADC_VOL, ADC_VREF_VCC)])); 
			}
			
			// FB Volume Pot ADC
			if(loop_counter==7) {
				//FBVR = PROGMEM_READ_BYTE(&(adc_to_wiper[adc_read_10bit(ADC_FBVOL, ADC_VREF_VCC)])); 
				FBVR = adc_read_8bit(ADC_FBVOL, ADC_VREF_VCC); 
			}
			
			// Encoder State Check(every 1msec Main Loop)
			switch(sample_encoder()) {
				case ENCODER_INCREMENT:
					if(!ioport_get_pin_level(SW0)) {
						switch(LCD_CMD) {
							case LCD_CMD_TRANSFER_REG:
								LCD_CMD = LCD_CMD_DISPLAY_SAVE;
							break;
							case LCD_CMD_DISPLAY_SAVE:
								LCD_CMD = LCD_CMD_DISPLAY_LOAD;
							break;
							case LCD_CMD_DISPLAY_LOAD:
								LCD_CMD = LCD_CMD_DISPLAY_VOLUME;
							break;
							case LCD_CMD_DISPLAY_VOLUME:
								LCD_CMD = LCD_CMD_TRANSFER_REG;
							break;
						}
					}
					else if((LCD_CMD==LCD_CMD_DISPLAY_LOAD)||(LCD_CMD==LCD_CMD_DISPLAY_SAVE)) {
						DCR1 = ((DCR1+(0x01<<MBANK0_BIT))&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT))) | (DCR1&~(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)));
					}
					else {
						switch(DCR1 & 0x01) {
							case DISPLAY_DELAY_TIME: 
								if(Get_Delay_Time_ms_to_Tempo(DTR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT)) > (MIN_DELAY_TEMPO-1)) {
									switch(((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT)) {
										case 0: 
											if(DTR!=1000) DTR++;	// Transfer Gotten Delay Time to Tempo;
										break;
										case 1: 
											if(DTR!=750) DTR++;	// Transfer Gotten Delay Time to Tempo;
										break;
										case 2:
											if(DTR!=500) DTR++;	// Transfer Gotten Delay Time to Tempo;
										break;
										case 3: 
											if(DTR!=375) DTR++; // Transfer Gotten Delay Time to Tempo;
										break; 
										case 4:
											if(DTR!=333) DTR++;	// Transfer Gotten Delay Time to Tempo;
										break;
										case 5:
											if(DTR!=166) DTR++;	// Transfer Gotten Delay Time to Tempo;
										break;
									}
								}
								TPR = Get_Delay_Time_ms_to_Tempo(DTR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));	// Transfer Gotten Delay Time to Tempo
							break;
							case DISPLAY_TEMPO:
								if(TPR<MAX_DELAY_TEMPO) TPR++;
								DTR = Get_Tempo_to_Delay_Time_ms(TPR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));
							break;
							default:
							break;
						}
					}
					break;
				case ENCODER_DECREMENT: 
					if(!ioport_get_pin_level(SW0)) {
						switch(LCD_CMD) {
							case LCD_CMD_TRANSFER_REG:
							LCD_CMD = LCD_CMD_DISPLAY_VOLUME;
							break;
							case LCD_CMD_DISPLAY_VOLUME:
							LCD_CMD = LCD_CMD_DISPLAY_LOAD;
							break;
							case LCD_CMD_DISPLAY_LOAD:
							LCD_CMD = LCD_CMD_DISPLAY_SAVE;
							break;
							case LCD_CMD_DISPLAY_SAVE:
							LCD_CMD = LCD_CMD_TRANSFER_REG;
							break;
							default:
							break;
						}
					}
					else if((LCD_CMD==LCD_CMD_DISPLAY_LOAD)||(LCD_CMD==LCD_CMD_DISPLAY_SAVE)) {
						DCR1 = ((DCR1-(0x01<<MBANK0_BIT))&(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT))) | (DCR1&~(get_bit_mask(MBANK0_BIT)|get_bit_mask(MBANK1_BIT)|get_bit_mask(MBANK2_BIT)));
					}
					else {
						switch(DCR1 & 0x01) {
							case DISPLAY_DELAY_TIME:
							if(Get_Delay_Time_ms_to_Tempo(DTR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT)) < MAX_DELAY_TEMPO) {	// Transfer Gotten Delay Time to Tempo;
								switch(((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT)) {
									case 0:
										if(DTR!=120) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break;
									case 1:
										if(DTR!=90) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break;
									case 2:
										if(DTR!=60) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break;
									case 3: 
										if(DTR!=45) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break; 
									case 4:
										if(DTR!=40) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break;
									case 5:
										if(DTR!=20) DTR--;	// Transfer Gotten Delay Time to Tempo;
									break;
								}
							}
							TPR = Get_Delay_Time_ms_to_Tempo(DTR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));	// Transfer Gotten Delay Time to Tempo
							break;
							case DISPLAY_TEMPO:
							if(TPR>MIN_DELAY_TEMPO) TPR--;
							DTR = Get_Tempo_to_Delay_Time_ms(TPR, ((DCR2&(get_bit_mask(DTYPE2_BIT)|get_bit_mask(DTYPE1_BIT)|get_bit_mask(DTYPE0_BIT)))>>DTYPE0_BIT));
							break;
							default:
							break;
						}
					}
					break;
				default: 
				break;
			}
			
			
			// Effect Mode Setting
			if((DCR0|(uint8_t)(~(get_bit_mask(INDET1_BIT)|get_bit_mask(INDET0_BIT))))==0xFF) {			// Stereo Input Mode
				set_bit_level_high(DCR1, MIXSEL0_BIT);
				set_bit_level_high(DCR1, MIXSEL1_BIT);
				set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
				FXVOLUME[0] = VOLR;
				FXVOLUME[1] = VOLR;
				FBVOLUME[0] = FBVR; 							// Feedback 2 -> 2 
				FBVOLUME[1]= FBVR;								// Feedback 1 -> 1
				FBVOLUME[2] = 0; 								// Feedback 1 -> 2
				FBVOLUME[3] = 0; 								// Feedback 2 -> 1
				//DCR1 = (DCR1 & 0x0F)|0xF0;					// For Debug
			}
			else if((DCR0|(uint8_t)(~(get_bit_mask(OUTDET1_BIT)|get_bit_mask(OUTDET0_BIT))))==0xFF) {		// Stereo Output Mode
				if(DCR0&get_bit_mask(DRY_MIX_BIT)) {						// DRY or Mix check
					// DRY mode
					switch(DCR0&(uint8_t)(get_bit_mask(INDET1_BIT)|get_bit_mask(INDET0_BIT))) {
						//case 0:
							// No Input Detect
						//break;
						case 2:
							// Input 1 Detect
							set_bit_level_high(DCR1, MIXSEL0_BIT);
							set_bit_level_high(DCR1, MIXSEL1_BIT);
							set_bit_level_high(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = 0;
							FXVOLUME[1] = VOLR;
							FBVOLUME[0] = 0; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = 0; 								// Feedback 1 -> 2
							FBVOLUME[3] = FBVR; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x00;					// For Debug
						break;
						case 8:
							// Input 2 Detect
							set_bit_level_low(DCR1, MIXSEL0_BIT);
							set_bit_level_low(DCR1, MIXSEL1_BIT);
							set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = 0;
							FXVOLUME[1] = VOLR;
							FBVOLUME[0] = FBVR; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = 0; 								// Feedback 1 -> 2
							FBVOLUME[3] = 0; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x10;					// For Debug
						break; 
						//case 10:
							// Input 1/2 Dtect
						//break;
						default: 
							// Exception
							set_bit_level_high(DCR1, MIXSEL0_BIT);
							set_bit_level_high(DCR1, MIXSEL1_BIT);
							set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = 0;
							FXVOLUME[1] = 0;
							FBVOLUME[0] = 0; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = 0; 								// Feedback 1 -> 2
							FBVOLUME[3] = 0; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x20;					// For Debug
						break; 
					}
				}
				else { 
					// MIX mode
					switch(DCR0&(uint8_t)(get_bit_mask(INDET1_BIT)|get_bit_mask(INDET0_BIT))) {
						//case 0:
							// No Input Detect
						//break;
						case 2:
							// Input 1 Detect
							set_bit_level_high(DCR1, MIXSEL0_BIT);
							set_bit_level_low(DCR1, MIXSEL1_BIT);
							set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = VOLR;
							FXVOLUME[1] = VOLR;
							FBVOLUME[0] = 0; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = 0; 								// Feedback 1 -> 2
							FBVOLUME[3] = FBVR; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x30;					// For Debug
						break;
						case 8:
							// Input 2 Detect
							set_bit_level_low(DCR1, MIXSEL0_BIT);
							set_bit_level_high(DCR1, MIXSEL1_BIT);
							set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = ((DCR0&get_bit_mask(ON_OFF_BIT))==get_bit_mask(ON_OFF_BIT)) ? VOLR : 0;
							FXVOLUME[1] = ((DCR0&get_bit_mask(ON_OFF_BIT))==get_bit_mask(ON_OFF_BIT)) ? VOLR : 0;
							FBVOLUME[0] = 0; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = FBVR; 								// Feedback 1 -> 2
							FBVOLUME[3] = 0; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x40;					// For Debug
						break;
						//case 10:
						// Input 1/2 Dtect
						//break;
						default:
							// Exception
							set_bit_level_high(DCR1, MIXSEL0_BIT);
							set_bit_level_high(DCR1, MIXSEL1_BIT);
							set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
							FXVOLUME[0] = 0;
							FXVOLUME[1] = 0;
							FBVOLUME[0] = 0; 							// Feedback 2 -> 2
							FBVOLUME[1] = 0;								// Feedback 1 -> 1
							FBVOLUME[2] = 0; 								// Feedback 1 -> 2
							FBVOLUME[3] = 0; 								// Feedback 2 -> 1
							//DCR1 = (DCR1 & 0x0F)|0x50;					// For Debug
						break;
					}
				}
			}
			else if((DCR0|(uint8_t)(~(get_bit_mask(OUTDET0_BIT))))==0xFF) {					// Out1 Only Detect
				switch(DCR0&(uint8_t)(get_bit_mask(INDET1_BIT)|get_bit_mask(INDET0_BIT))) {
					//case 0:
						// No Input Detect
					//break;
					case 2:
						// Input 1 Detect
						set_bit_level_high(DCR1, MIXSEL0_BIT);
						set_bit_level_high(DCR1, MIXSEL1_BIT);
						set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = VOLR;
						FXVOLUME[1] = 0;
						FBVOLUME[0] = 0; 							// Feedback 2 -> 2
						FBVOLUME[1] = FBVR;								// Feedback 1 -> 1
						FBVOLUME[2] = 0; 								// Feedback 1 -> 2
						FBVOLUME[3] = 0; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0x60;					// For Debug
					break;
					case 8:
						// Input 2 Detect
						set_bit_level_low(DCR1, MIXSEL0_BIT);
						set_bit_level_high(DCR1, MIXSEL1_BIT);
						set_bit_level_high(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = ((DCR0&get_bit_mask(ON_OFF_BIT))==get_bit_mask(ON_OFF_BIT)) ? VOLR : 0;
						FXVOLUME[1] = 0;
						FBVOLUME[0] = 0; 							// Feedback 2 -> 2
						FBVOLUME[1] = 0;								// Feedback 1 -> 1
						FBVOLUME[2] = FBVR; 								// Feedback 1 -> 2
						FBVOLUME[3] = 0; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0x70;					// For Debug
					break;
					//case 10:
						// Input 1/2 Dtect
					//break;
					default:
						// Exception
						set_bit_level_high(DCR1, MIXSEL0_BIT);
						set_bit_level_high(DCR1, MIXSEL1_BIT);
						set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = 0;
						FXVOLUME[1] = 0;
						FBVOLUME[0] = 0; 							// Feedback 2 -> 2
						FBVOLUME[1] = 0;								// Feedback 1 -> 1
						FBVOLUME[2] = 0; 								// Feedback 1 -> 2
						FBVOLUME[3] = 0; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0x80;					// For Debug
					break;
				}
			}
			else if((DCR0|(uint8_t)(~(get_bit_mask(OUTDET1_BIT))))==0xFF) {					// Out2 Only Detect
				switch(DCR0&(uint8_t)(get_bit_mask(INDET1_BIT)|get_bit_mask(INDET0_BIT))) {
					//case 0:
						// No Input Detect
					//	break;
					case 2:
						// Input 1 Detect
						set_bit_level_high(DCR1, MIXSEL0_BIT);
						set_bit_level_low(DCR1, MIXSEL1_BIT);
						set_bit_level_high(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = 0;
						FXVOLUME[1] = VOLR;
						FBVOLUME[0] = 0; 							// Feedback 2 -> 2
						FBVOLUME[1] = 0;								// Feedback 1 -> 1
						FBVOLUME[2] = 0; 								// Feedback 1 -> 2
						FBVOLUME[3] = FBVR; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0x90;					// For Debug
					break;
					case 8:
						// Input 2 Detect
						set_bit_level_high(DCR1, MIXSEL0_BIT);
						set_bit_level_high(DCR1, MIXSEL1_BIT);
						set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = 0;
						FXVOLUME[1] = ((DCR0&get_bit_mask(ON_OFF_BIT))==get_bit_mask(ON_OFF_BIT)) ? VOLR : 0;
						FBVOLUME[0] = FBVR; 							// Feedback 2 -> 2
						FBVOLUME[1] = 0;								// Feedback 1 -> 1
						FBVOLUME[2] = 0; 								// Feedback 1 -> 2
						FBVOLUME[3] = 0; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0xA0;					// For Debug
					break;
					//case 10:
						// Input 1/2 Dtect
					//break;
					default:
						// Exception
						set_bit_level_high(DCR1, MIXSEL0_BIT);
						set_bit_level_high(DCR1, MIXSEL1_BIT);
						set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
						FXVOLUME[0] = 0;
						FXVOLUME[1] = 0;
						FBVOLUME[0] = 0; 							// Feedback 2 -> 2
						FBVOLUME[1] = 0;								// Feedback 1 -> 1
						FBVOLUME[2] = 0; 								// Feedback 1 -> 2
						FBVOLUME[3] = 0; 								// Feedback 2 -> 1
						//DCR1 = (DCR1 & 0x0F)|0xB0;					// For Debug
					break;
				}
			}
			else {
				set_bit_level_high(DCR1, MIXSEL0_BIT);
				set_bit_level_high(DCR1, MIXSEL1_BIT);
				set_bit_level_low(DCR1, BSN_BIT);			// Set 3207 Stage Number
				FXVOLUME[0] = 0;
				FXVOLUME[1] = 0;
				FBVOLUME[0] = 0; 							// Feedback 2 -> 2
				FBVOLUME[1] = 0;								// Feedback 1 -> 1
				FBVOLUME[2] = 0; 								// Feedback 1 -> 2
				FBVOLUME[3] = 0; 								// Feedback 2 -> 1
				//DCR1 = (DCR1 & 0x0F)|0xC0;					// For Debug
			}
			
			// Set Effect On/Off
			ioport_set_pin_level(ON_OFF, DCR0&get_bit_mask(ON_OFF_BIT));	// Set ON/OFF Switch(U203:ADG621) ON
			
			// Delay Time Setting
			i=0;
			do {
				i++;
				//OCR = (uint16_t)((uint32_t)(DTR * 64000UL)>>(10+(TCCR1B&0x07)+((DCR1&0x08)>>3)));
				OCR = (uint16_t)((uint32_t)(DTR * 64000UL)>>(10+i+(((DCR1&0x08)>>3)+1)+1));
			} while(!(OCR<0x03FF));
			TCCR1B = (DCR0&get_bit_mask(ON_OFF_BIT)) ? (TCCR1B&~0x0F) | i : TCCR1B&~0x0F;
			TC1H = OCR >> 8;
			OCR1C = (uint8_t) OCR;
			OCR = OCR >> 1;
			TC1H = OCR >> 8;
			OCR1A = (uint8_t) OCR;
						
			// Set Data to AD5282
			if(loop_counter==8) {
				//SetAD5282();
				//for(i=0; i<2; i++) {
					TWI_targetSlaveAddress = AD5280_5282_ADR_BYTE;
					messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
					messageBuf[1] = ((i2c_send_time_counter&0x01)<<AD5280_5282_SUBADR_BIT);             // The first byte is used for commands.
					messageBuf[2] = (unsigned char)((uint8_t)(0xFF)-FXVOLUME[i2c_send_time_counter&0x01]);
					temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 3);
					if (!temp)    // One of the operations failed.
					{             // Use TWI status information to detemine cause of failure and take appropriate actions.
						TWI_Act_On_Failure_In_Last_Transmission( USI_TWI_Get_State_Info( ) );
						asm("nop"); // Put own code here.
					}
				//}
			}
			
			// Set Data to AD5263
			if(loop_counter==9) {
				// Set Data to AD5263
				//for(i=0; i<4; i++) {
					//SetAD5263();
					TWI_targetSlaveAddress = AD5263_ADR_BYTE;
					messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
					messageBuf[1] = ((i2c_send_time_counter&0x03)<<AD5263_ADR0_BIT | (DCR1 & (get_bit_mask(AD5263_O1_BIT) | get_bit_mask(AD5263_O2_BIT))));             // The first byte is used for commands.
					messageBuf[2] = (unsigned char)(FBVOLUME[i2c_send_time_counter&0x03]);
					temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 3);
					if (!temp)    // One of the operations failed.
					{             // Use TWI status information to detemine cause of failure and take appropriate actions.
						TWI_Act_On_Failure_In_Last_Transmission( USI_TWI_Get_State_Info( ) );
						asm("nop"); // Put own code here.
					}
				//}
				i2c_send_time_counter++;
			}
			
			// Main Loop finish flow
			TIFR |= 1 << OCF0A;						// Clear Timer0 Interrupt
			if(loop_counter==9) {
				loop_counter = 0;					// Clear Loop Counter
			}
			else {
				loop_counter++;						// Increment Loop Counter
			}
		}
	}
}
