#include "LiquidCrystal_Config.h"
#include "LiquidCrystal.h"


#define LCD_SET_RS_BIT		(LCD_CMND_PRT |= (1<<LCD_RS))
#define LCD_CLR_RS_BIT      (LCD_CMND_PRT &= ~(1<<LCD_RS))

#define LCD_SET_RW_BIT      (LCD_CMND_PRT |= (1<<LCD_RW))
#define LCD_CLR_RW_BIT      (LCD_CMND_PRT &= ~(1<<LCD_RW))

#define LCD_SET_EN_BIT      (LCD_CMND_PRT |= (1<<LCD_EN))
#define LCD_CLR_EN_BIT      (LCD_CMND_PRT &= ~(1<<LCD_EN))


uint8_t _row_offsets[4] = {0x80, 0xC0, 0x94, 0xD4};
	
/*********** Mid level commands, for sending data/cmd */
inline static void lcd_command(unsigned char cmd);
inline static void lcd_write(unsigned char value);

/************ Low level data pushing commands **********/
static void send(unsigned char value, uint8_t mdoe);
static void pulse_enable(void);
static void write_4_bit(unsigned char value);
static void write_8_bit(unsigned char value);



/**
 * 
 **/
void liquidCrystal_init()
{

    #if LCD_CTRL_MODE == 4
        LCD_DATA_DDR = (1<<LCD_D0) | (1<<LCD_D1) | (1<<LCD_D2) | (1<<LCD_D3);
    #else
        LCD_DATA_DDR = (1<<LCD_D0) | (1<<LCD_D1) | (1<<LCD_D2) | (1<<LCD_D3) |
                       (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);
    #endif

    // we can save 1 pin by not using RW. Indicate by passing 255 instead of pin#
    #if LCD_RW != 255
        LCD_CMND_DDR |= (1<<LCD_RS) | (1<<LCD_RW) | (1<<LCD_EN);
    #else
        LCD_CMND_DDR |= (1<<LCD_RS) | (1<<LCD_EN);
    #endif

    #if LCD_CTRL_MODE == 4
        _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    #else
        _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
    #endif

    #if(LCD_ROW_COUNT > 1)
        _displayfunction |= LCD_2LINE;
	#endif
    _numlines = LCD_ROW_COUNT;

    // for some 1 line displays you can select a 10 pixel high font
    #if ((LCD_DOT_SIZE != LCD_5x8DOTS) && (LCD_ROW_COUNT == 1)) 
        _displayfunction |= LCD_5x10DOTS;
    #endif


    // Now we pull both RS and R/W low to begin commands
    LCD_CLR_RS_BIT;
    LCD_CLR_EN_BIT;
    #if LCD_RW != 255
        LCD_CLR_RW_BIT;
    #endif	

    //put the LCD into 4 bit or 8 bit mode
    #if LCD_CTRL_MODE == 4
        // we start in 8bit mode, try to set 4 bit mode
        write_4_bit(0x03);
        _delay_us(4500);    // wait min 4.1ms
        
        // second try
        write_4_bit(0x03);
        _delay_us(4500);    // wait min 4.1ms
        
        // third go!
        write_4_bit(0x03);
        _delay_us(150);

        // finally, set to 4-bit interface
        write_4_bit(0x02);
    #else
         // Send function set command sequence
        lcd_command(LCD_FUNCTIONSET | _displayfunction);
        _delay_us(4500);

        // second try
        lcd_command(LCD_FUNCTIONSET | _displayfunction);
        _delay_us(150);

        // third go
        lcd_command(LCD_FUNCTIONSET | _displayfunction);

    #endif

    // finally, set # lines, font size, etc.
    lcd_command(LCD_FUNCTIONSET | _displayfunction);

    // turn the display on with no cursor or blinging default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF;
    liquidCrystal_display();

    // clear it off
    liquidCrystal_clear();
    
    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    lcd_command(LCD_ENTRYMODESET | _displaymode);	
}


/********** high level commands, for the user! */
void liquidCrystal_clear()
{
    lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
    _delay_us(2000);                // this command takes a long time!
}

//*******************************************************
void liquidCrystal_home()
{
  lcd_command(LCD_RETURNHOME);      // set cursor position to zero
  _delay_us(2000);                  // this command takes a long time!
}



//*******************************************************
void liquidCrystal_setCursor(uint8_t col, uint8_t row)
{
    const uint8_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
    if ( row >= max_lines ) 
        row = max_lines - 1;    // we count rows starting w/0
    if ( row >= _numlines ) 
        row = _numlines - 1;    // we count rows starting w/0

    lcd_command(LCD_SETDDRAMADDR | (_row_offsets[row] + col));
}

//*******************************************************
// Turn the display on/off (quickly)
void liquidCrystal_noDisplay() 
{
    _displaycontrol &= ~LCD_DISPLAYON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void liquidCrystal_display()
{
    _displaycontrol |= LCD_DISPLAYON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

//*******************************************************
// Turns the underline cursor on/off
void liquidCrystal_noCursor() 
{
    _displaycontrol &= ~LCD_CURSORON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void liquidCrystal_cursor() 
{
    _displaycontrol |= LCD_CURSORON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

//*******************************************************
// Turn on and off the blinking cursor
void liquidCrystal_noBlink() 
{
    _displaycontrol &= ~LCD_BLINKON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void liquidCrystal_blink() 
{
    _displaycontrol |= LCD_BLINKON;
    lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

//*******************************************************
// These commands scroll the display without changing the RAM
void liquidCrystal_scrollDisplayLeft(void) 
{
    lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void liquidCrystal_scrollDisplayRight(void) 
{
    lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

//*******************************************************
// This is for text that flows Left to Right
void liquidCrystal_leftToRight(void) 
{
    _displaymode |= LCD_ENTRYLEFT;
    lcd_command(LCD_ENTRYMODESET | _displaymode);
}

//*******************************************************
// This is for text that flows Right to Left
void liquidCrystal_rightToLeft(void) 
{
    _displaymode &= ~LCD_ENTRYLEFT;
    lcd_command(LCD_ENTRYMODESET | _displaymode);
}

//*******************************************************
// This will 'right justify' text from the cursor
void liquidCrystal_autoscroll(void) 
{
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_command(LCD_ENTRYMODESET | _displaymode);
}

//*******************************************************
// This will 'left justify' text from the cursor
void liquidCrystal_noAutoscroll(void) 
{
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_command(LCD_ENTRYMODESET | _displaymode);
}

//*******************************************************
// Allows us to fill the first 8 CGRAM locations
// with custom characters
void liquidCrystal_createChar(uint8_t location, unsigned char charmap[]) 
{
    location &= 0x7; // we only have 8 locations 0-7
    lcd_command(LCD_SETCGRAMADDR | (location << 3));
    for (int i=0; i<8; i++) 
        lcd_write(charmap[i]);
}


/*********** mid level commands, for sending data/CMDs */
inline static void lcd_command(unsigned char cmd)
{
    send(cmd, LOW);
}

inline static void lcd_write(unsigned char value)
{
    send(value, HIGH);
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
static void send(unsigned char value, uint8_t mode)
{
     (mode == LOW) ? LCD_CLR_RS_BIT : LCD_SET_RS_BIT;

    // if there is a RW pin indicated, set it low to Write
    #if LCD_RW != 255
        LCD_CLR_RW_BIT;
    #endif

    #if LCD_CTRL_MODE != 4
        write_8_bit(value);
    #else
        write_4_bit(value>>4);      // MSB
        write_4_bit(value & 0x0F);  // LSB
    #endif
}

//*******************************************************
static void pulse_enable(void)
{
    LCD_CLR_EN_BIT;
    _delay_us(1);
    LCD_SET_EN_BIT;
    _delay_us(1);       // enable pulse must be >450ns
    LCD_CLR_EN_BIT;
    _delay_us(100);     // commands need > 37us to settle
}

//*******************************************************
static void write_4_bit(unsigned char value)
{
    LCD_DATA_PRT &= ~((1<<LCD_D0) | (1<<LCD_D1) | (1<<LCD_D2) | (1<<LCD_D3));
    LCD_DATA_PRT |= (value & 0x01) ? (1<<LCD_D0) : 0x00;
    LCD_DATA_PRT |= (value & 0x02) ? (1<<LCD_D1) : 0x00;
    LCD_DATA_PRT |= (value & 0x04) ? (1<<LCD_D2) : 0x00;
    LCD_DATA_PRT |= (value & 0x08) ? (1<<LCD_D3) : 0x00;

    pulse_enable();
}
//*******************************************************
static void write_8_bit(unsigned char value)
{
    LCD_DATA_PRT = value;

    pulse_enable();
}

//*******************************************************
void liquidCrystal_print( char * str )
{
    unsigned char i = 0 ;
    while(str[i]!=0)
    {
        lcd_write(str[i]);
        i++ ;
    }
}
