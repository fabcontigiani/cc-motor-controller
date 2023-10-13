#ifndef _LIQUIDCRYSTAL_H
#define _LIQUIDCRYSTAL_H

#include <stdint.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define HIGH 0x01
#define LOW  0x00

/*
	uint8_t _rs_pin; // LOW: command.  HIGH: character.
	uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
	uint8_t _enable_pin; // activated by a HIGH pulse.
	uint8_t _data_pins[8];
*/

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;

uint8_t _numlines;
extern uint8_t _row_offsets[4];		/* {0x80, 0xC0, 0x94, 0xD4} */



/********** high level commands, for the user! ***************/
void liquidCrystal_init();
void liquidCrystal_print( char * str );
void liquidCrystal_clear();
void liquidCrystal_home();
void liquidCrystal_setCursor(uint8_t col, uint8_t row);
void liquidCrystal_noDisplay();
void liquidCrystal_display();
void liquidCrystal_noCursor();
void liquidCrystal_cursor();
void liquidCrystal_noBlink();
void liquidCrystal_blink();
void liquidCrystal_scrollDisplayLeft(void);
void liquidCrystal_scrollDisplayRight(void);
void liquidCrystal_leftToRight(void);
void liquidCrystal_rightToLeft(void);
void liquidCrystal_autoscroll(void);
void liquidCrystal_noAutoscroll(void);
void liquidCrystal_createChar(uint8_t location, unsigned char charmap[]);

#endif