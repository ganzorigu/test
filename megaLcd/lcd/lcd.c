/****************************************************************************
 Title:     HD44780U LCD library
 Author:    Peter Fleury <pfleury@gmx.ch>  http://tinyurl.com/peterfleury
 File:	    $Id: lcd.c,v 1.15.2.2 2015/01/17 12:16:05 peter Exp $
 Software:  AVR-GCC 3.3 
 Target:    any AVR device, memory mapped mode only for AT90S4414/8515/Mega

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text lcd display

       Originally based on Volker Oth's lcd library,
       changed lcd_init(), added additional constants for lcd_command(),
       added 4-bit I/O mode, improved and optimized code.

       Library can be operated in memory mapped mode (LCD_IO_MODE=0) or in 
       4-bit IO port mode (LCD_IO_MODE=1). 8-bit IO port mode not supported.
       
       Memory mapped mode compatible with Kanda STK200, but supports also
       generation of R/W signal through A8 address line.

 USAGE
       See the C include lcd.h file for a description of each function
       
*****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd.h"
#include "../i2c/i2cmaster.h"
//#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );

#define pcf8574 0x3f<<1


/* 
** constants/macros 
*/
#define DDR(x) (*(&x - 1))      /* address of data direction register of port x */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
    /* on ATmega64/128 PINF is on port 0x00 and not 0x60 */
    #define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
#else
	#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
#endif



#define lcd_e_delay()   _delay_us(LCD_DELAY_ENABLE_PULSE);
#define lcd_e_toggle()  toggle_e()




#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 


/* 
** function prototypes 
*/
#if LCD_IO_MODE
static void toggle_e(void);
#endif

/*
** local functions
*/


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delay_us(us) 
volatile uint8_t pcf_data=0x00;

void IOEwrite(uint8_t data)
{
	i2c_Start();
	i2c_AddWrite(pcf8574);
	i2c_DataWrite(data);
	i2c_Stop();
}

void lcd_control_pins(uint8_t pin, uint8_t iolevel)
{
	if (iolevel)
	{
		pcf_data |= (1<<pin);
	}
	else
	{
		pcf_data &= ~(1<<pin);
	}
	IOEwrite(pcf_data);
}


/* toggle Enable Pin to initiate write */
static void toggle_e(void)
{
    //lcd_e_high();
	lcd_control_pins(LCD_E_PIN,HIGH);
    lcd_e_delay();
    //lcd_e_low();
	lcd_control_pins(LCD_E_PIN,LOW);
}



/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data    
                 0: write instruction
Returns:  none
*************************************************************************/

static void lcd_write(uint8_t data,uint8_t rs) 
{   
	
	if (rs)
	{
		//lcd_control_pins(LCD_RS_PIN,HIGH);
		pcf_data|=_BV(LCD_RS_PIN);
	}
	else
	{
		//lcd_control_pins(LCD_RS_PIN,LOW);
		pcf_data&=~_BV(LCD_RS_PIN);
	}	
	//lcd_control_pins(LCD_RW_PIN,LOW);
	pcf_data&=~_BV(LCD_RW_PIN);
	// send high nibble first
	pcf_data &= 0x0F;
	pcf_data |= (data & 0xF0);
	pcf_data |= (1<<LCD_E_PIN);	// E pin high
	IOEwrite(pcf_data);
	pcf_data &= ~(1<<LCD_E_PIN);	// E pin low
	IOEwrite(pcf_data);
	// send low nibble second
	pcf_data &= 0x0F;
	pcf_data |= ((data<<4) & 0xF0);
	pcf_data |= (1<<LCD_E_PIN);	// E pin high
	IOEwrite(pcf_data);
	pcf_data &= ~(1<<LCD_E_PIN);	// E pin low
	IOEwrite(pcf_data);	
}


/*
** PUBLIC FUNCTIONS 
*/

/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t cmd)
{    
    lcd_write(cmd,0);
}


/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t data)
{    
    lcd_write(data,1);
}



/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
    if ( y==0 ) 
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
}/* lcd_gotoxy */


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(1<<LCD_CLR);
	_delay_ms(1);
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
}


/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{    
    lcd_write(c, 1);    

}/* lcd_putc */


/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */


/*************************************************************************
Display string from program memory without auto linefeed 
Input:     string from program memory be be displayed                                        
Returns:   none
*************************************************************************/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_putc(c);
    }

}/* lcd_puts_p */


/*************************************************************************
Initialize display and select type of cursor 
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/
void lcd_init(uint8_t dispAttr)
{	
	pcf_data = 0;
	i2c_Init();	
	
	lcd_control_pins(3,HIGH);
	
    delay(LCD_DELAY_BOOTUP);             /* wait 16ms or more after power-on       */
    
    /* initial write to lcd is 8bit */
	pcf_data |= (_BV(LCD_DATA1_PIN) | _BV(LCD_DATA0_PIN));
	IOEwrite(pcf_data);
		
    lcd_e_toggle();
    delay(LCD_DELAY_INIT);               /* delay, busy flag can't be checked here */
   
    /* repeat last command */ 
    lcd_e_toggle();      
    delay(LCD_DELAY_INIT_REP);           /* delay, busy flag can't be checked here */
    
    /* repeat last command a third time */
    lcd_e_toggle();      
    delay(LCD_DELAY_INIT_REP);           /* delay, busy flag can't be checked here */

    /* now configure for 4bit mode */		
	pcf_data &= ~_BV(LCD_DATA0_PIN);	// LCD_FUNCTION_4BIT_1LINE>>4
	IOEwrite(pcf_data);
    lcd_e_toggle();
    delay(LCD_DELAY_INIT_4BIT);          /* some displays need this additional delay */
    
    /* from now the LCD only accepts 4 bit I/O, we can use lcd_command() */    

    lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */

    lcd_command(LCD_DISP_ON);              /* display off                  */
    lcd_clrscr();                           /* display clear                */ 
	
    lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
    lcd_command(dispAttr);                  /* display/cursor control       */

}/* lcd_init */
