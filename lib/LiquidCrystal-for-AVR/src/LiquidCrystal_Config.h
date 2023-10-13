#ifndef _LIQUIDCRYSTAL_CONFIG_H
#define _LIQUIDCRYSTAL_CONFIG_H

#include <avr/io.h>
#include <util/delay.h>

#define LCD_DATA_DDR DDRB
#define LCD_DATA_PRT PORTB
#define LCD_DARA_PIN PINB

#define LCD_CMND_DDR DDRB
#define LCD_CMND_PRT PORTB
#define LCD_CMND_PIN PINB

#define LCD_COL_COUNT 16
#define LCD_ROW_COUNT 2

#define LCD_CTRL_MODE 4     // Use only 4 bit or 8 bit values

#define LCD_DOT_SIZE 0

#define LCD_RS 0
#define LCD_RW 255            // if LCD_RW = 255, a pin can be saved.
#define LCD_EN 1

#define LCD_D0 2
#define LCD_D1 3
#define LCD_D2 4
#define LCD_D3 5
#if LCD_CTRL_MODE == 8
	#define LCD_D4
	#define LCD_D5
	#define LCD_D6
	#define LCD_D7
#endif

#endif