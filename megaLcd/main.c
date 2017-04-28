/*
 * megaLcd.c
 *
 * Created: 4/17/2017 10:17:05 AM
 * Author : ganzorig.u
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "misc/misc.h"
#include "lcd/lcd.h"




const uint8_t Symbol[] PROGMEM =
{
	0x02, 0xfd, 'o',	// ok
	0x62, 0x9D, 'u',	// up
	0x22, 0xDD, 'l',	// left
	0xc2, 0x3d, 'r',	// right
	0xa8, 0x57, 'd',	// down	
	0x68, 0x97, '1',
	0x98, 0x67, '2',
	0xb0, 0x4F, '3',
	0x30, 0xCF, '4',
	0x18, 0xE7, '5',
	0x7A, 0x85, '6',
	0x10, 0xEF, '7',
	0x38, 0xC7, '8',
	0x5A, 0xA5, '9',
	0x4A, 0xB5, '0',
	0x42, 0xBD, '*',
	0x52, 0xAD, '#'
	
	
};

uint8_t sValue = 0;
uint8_t IrBytes[4];
uint8_t sCount = 0;
uint8_t UserInput[4];



enum Alarm_state {DISARMED, ARMED};
enum Alarm_state SM; 


volatile unsigned char genflg = 0;
#define bIrRecvd	0
#define bIrRep		1
#define bNewByte	2


// variables used for IR
volatile uint16_t irBuff[67];
volatile uint8_t pulseIndex=0;

// variables used for UART
#define UART_TX0_BUFFER_SIZE 128
#define UART_TX0_BUFFER_MASK ( UART_TX0_BUFFER_SIZE - 1)

#define USART_BAUD  9600L
#define UBRR_VAL ((unsigned int)(F_CPU/(16*USART_BAUD)-1))
#define cBs 0x08		// backspace
#define ccr 0x0D		// carriage return

static volatile char UsartTxBuff[UART_TX0_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

volatile unsigned char UsartTxPtr = 0;


char UsartRxBuff[30];
volatile unsigned char UsartRxPtr = 0;
volatile char Fifo[32];
volatile unsigned char FifoWr=0, FifoRd=0;

void Timer1_start()
{
	TCCR1B = (1<<CS11)|(1<<CS10);				// set the prescaler to 64
}


void Timer1_stop()
{
	TCCR1B &= ~((1<<CS11)|(1<<CS10));				//
}


//************************* INTERRUPT ROUTINES START **********************************

ISR(TIMER1_OVF_vect)
{
	Timer1_stop();
	TCNT1 = 18000/4;
}


ISR(INT5_vect)
{
	Timer1_stop();
	uint16_t timerVal = TCNT1;
	TCNT1 = 0;
	

	switch(pulseIndex)
	{
		case 0:
		if (timerVal > 8900/4 && (PINE&(1<<PE5))!=0)
		{
			irBuff[pulseIndex] = timerVal;
			pulseIndex++;
		}
		break;
		case 1:
		if (timerVal > 4490/4)
		{
			irBuff[pulseIndex] = timerVal;
			pulseIndex++;
		}
		else if (timerVal > 2200/4)
		{
			pulseIndex = 0;
			genflg |=(1<<bIrRep);
		}
		break;
		default:
		if (pulseIndex > 66)
		{
			pulseIndex = 0;
			genflg |=(1<<bIrRecvd);
		}
		else
		{
			irBuff[pulseIndex] = timerVal;
			pulseIndex++;
		}
		break;
	}
	Timer1_start();
}


ISR(USART0_RX_vect){
	
	Fifo[FifoWr++]=UDR0;
	
	if (FifoWr > 31)
	{
		FifoWr = 0;
	}
	genflg |= (1<<bNewByte);
}

// USART0 data empty interrupt routine
ISR(USART0_UDRE_vect)
{
	unsigned char tmptail;
	
	if (UART_TxHead != UART_TxTail) {
		tmptail = (UART_TxTail + 1) & UART_TX0_BUFFER_MASK;
		UART_TxTail = tmptail;
		UDR0 = UsartTxBuff[tmptail]; // Start the transmission
	}
	else {
		UCSR0B &= ~(1<<UDRIE0);	// disable data empty interrupt
	}
}
//************************* INTERRUPT ROUTINES END **********************************
void Usart0_Init();
void print_variable(int32_t var);
void print_hex(unsigned char * buff, unsigned char length);
void print_console(const char *text);
void uart0_putc(char data);
void Variable_Init();
void Timer1_Init();
void ExtPin_Init();
void lcd_hex(unsigned char * buff, unsigned char length);
uint8_t detect_symbol(uint8_t *a);
uint8_t read_Ir();
uint8_t check_pass();

int main(void)
{
    /* Replace with your application code */
	cli();	
	Variable_Init();
	Usart0_Init();	
	Timer1_Init();
	ExtPin_Init();
	
	lcd_init(LCD_DISP_ON);
	_delay_ms(100);	
	
	
	sei();		
    while (1) 
    {	
		if (genflg & (1<<bIrRecvd))
		{
			genflg &= ~(1<<bIrRecvd);			
			if (read_Ir())
			{
				lcd_gotoxy(15,0);
				lcd_putc(sValue);
				
				if (sValue=='o')
				{					
					
					if (check_pass()==1 && sCount==4)
					{
						lcd_gotoxy(0,1);
						if (SM==DISARMED)
						{							
							lcd_puts("ARMED");
							SM = ARMED;							
						}
						else
						{
							lcd_puts_p(PSTR("     "));
							SM = DISARMED;
						}
					}					
					sCount = 0;
					lcd_gotoxy(0,0);
					lcd_puts_p(PSTR("     "));
				}
				else
				{
					if(sCount < 4)
					{
						lcd_gotoxy(sCount,0);
						lcd_putc('*');	
						UserInput[sCount] = sValue;
						sCount++;					
					}

					
				}				
			}
		}
		else if (genflg & (1<<bIrRep))
		{
			genflg &= ~(1<<bIrRep);
			print_console(PSTR("Rep.\r\n"));
		}	
		
    }
}


void Timer1_Init()
{
	TCCR1B = (1<<CS11)|(1<<CS10);
	TIMSK1 = (1<<TOIE1);			// enable Timer1 overflow
}


void ExtPin_Init()
{
	EICRB = (1<<ISC50);	// any logical change on INT5
	EIMSK = (1<<INT5);	// Enable INT5
}



void Usart0_Init()
{
	UBRR0H = (unsigned char)(UBRR_VAL>>8);
	UBRR0L = (unsigned char)UBRR_VAL;
	// Enable receiver and Transmitter
	// Enable Rx Complete Interrupt Enable
	// Enable Data Register Empty interrupt Enable
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//|(1<<UDRIE0);
	
	UCSR0C = (3<<UCSZ10);
}


void uart0_putc(char data)
{
	unsigned char tmphead;

	tmphead  = (UART_TxHead + 1) & UART_TX0_BUFFER_MASK;

	while ( tmphead == UART_TxTail ) {
		;/* wait for free space in buffer */
	}

	UsartTxBuff[tmphead] = data;
	UART_TxHead = tmphead;

	/* enable UDRE interrupt */
	UCSR0B |= (1<<UDRIE0);

} /* uart0_putc */



//Print string stored in Program memory
void print_console(const char *text)
{
	char tempch;
	tempch = pgm_read_byte(text++);
	while(tempch!=0)
	{
		uart0_putc(tempch);
		tempch = pgm_read_byte(text++);
	}
}

void print_hex(unsigned char * buff, unsigned char length)
{
	unsigned char hex_data[2];
	for (unsigned char i=0; i < length; i++)
	{
		convert2hex(*buff,&hex_data[0]);
		uart0_putc(hex_data[0]);
		uart0_putc(hex_data[1]);
		uart0_putc(' ');
		buff++;
	}
	uart0_putc(ccr);
}

void lcd_hex(unsigned char * buff, unsigned char length)
{
	unsigned char hex_data[2];
	if (length < 5)
	{
		for (unsigned char i=0; i < length; i++)
		{
			convert2hex(*buff,&hex_data[0]);
			lcd_putc(hex_data[0]);
			lcd_putc(hex_data[1]);
			lcd_putc(' ');
			buff++;
		}
		uart0_putc(ccr);
	}
}


void print_variable(int32_t var)
{
	char str[16];
	char *pt_str;
	itoa(var, str, 10);
	pt_str = str;
	while(*pt_str!=0)
	{
		uart0_putc(*pt_str);
		pt_str++;
	}
}

void Variable_Init()
{
	UsartRxPtr = 0;
	UART_TxHead = 0;
	UART_TxTail = 0;
	SM = DISARMED;
}

uint8_t detect_symbol(uint8_t *a)
{
	uint8_t i = 0;	
	uint8_t HighByte = *a;
	a++;
	uint8_t LowByte = *a;
	
	while(i<17)
	{
		if (HighByte == pgm_read_byte(&Symbol[i*3]))
		{
			if (LowByte == pgm_read_byte(&Symbol[i*3+1]))
			{				
				sValue = pgm_read_byte(&Symbol[i*3+2]);
				return 1;
			}
		}
		i++;
	}	
	return 0;
}

uint8_t read_Ir()
{
	uint8_t i = 0, j=3;	
	uint8_t arr[4] = {0,0,0,0};
	unsigned char temp;
	while(i<32)
	{
		temp = i >> 3;
		arr[temp] = arr[temp] << 1;
		if (irBuff[j]>200)
		{
			arr[temp] |= 0x01;
		}
		i++;
		j=j+2;
	}
	IrBytes[0] = arr[0];
	IrBytes[1] = arr[1];
	IrBytes[2] = arr[2];
	IrBytes[3] = arr[3];
	return detect_symbol(&arr[2]);	
}


uint8_t check_pass()
{
	uint8_t pass[4]={'1', '2', '3', '4'};
	uint8_t i=0;
	
	for (i=0;i<4;i++)
	{
		if (UserInput[i]!=pass[i])
		{
			return 0;
		}
	}				
	return 1;
}