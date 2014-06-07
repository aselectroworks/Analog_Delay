/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include "ioport.h"

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	
	/* I/O Port confuguration */
	ioport_configure_pin(INDET0, IOPORT_DIR_INPUT);
	ioport_configure_pin(INDET1, IOPORT_DIR_INPUT);
	ioport_configure_pin(OUTDET0, IOPORT_DIR_INPUT);
	ioport_configure_pin(OUTDET1, IOPORT_DIR_INPUT);
	ioport_configure_pin(SW0, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(SW1, IOPORT_DIR_INPUT | IOPORT_PULL_UP);

	ioport_configure_pin(CLK_N, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(CLK_P, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(ENC_A, IOPORT_DIR_INPUT);
	ioport_configure_pin(ENC_B, IOPORT_DIR_INPUT);
	ioport_configure_pin(VOL, IOPORT_DIR_INPUT);
	ioport_configure_pin(FBVOL, IOPORT_DIR_INPUT);
	ioport_configure_pin(ON_OFF, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(SCL, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(SDA, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	
		
}
