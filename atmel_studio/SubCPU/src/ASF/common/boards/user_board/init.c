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
	ioport_configure_group(DB, 0xFF, IOPORT_DIR_OUTPUT |  IOPORT_INIT_LOW);
	ioport_configure_pin(RS, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(RW, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(E, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(SDA, IOPORT_DIR_INPUT | IOPORT_PULL_DOWN);
	ioport_configure_pin(SCL, IOPORT_DIR_INPUT | IOPORT_PULL_DOWN);
	
	
}
