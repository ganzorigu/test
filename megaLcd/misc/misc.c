/*
 * misc.c
 *
 * Created: 12/11/2016 12:11:37 AM
 *  Author: ganzo
 */ 

#include <avr/pgmspace.h>
#include "misc.h"

unsigned char databyte_cmp(unsigned char *in1, unsigned char *in2, unsigned char len)
{
	unsigned char i=0;
	for (i=0;i<len;i++)
	{
		if (*in1++!=*in2++)
		{
			return 0;
		}
	}
	return 1;
}

// search if in1 is found in in2
unsigned char cmd_cmp(const char *in1, char *in2)
{
	char temp_data;
	while(*in2!=0)	// loop until null
	{
		temp_data = pgm_read_byte(in1++);
		if (temp_data==0)
		{
			return 1;
		}
		else if (temp_data!=*(in2++))
		{
			return 0;
		}
	}
	return 1;
}

unsigned char convert2bcd(unsigned char temp)
{
	unsigned char t1 = 0;	// tenth
	
	while(temp>10)
	{
		temp = temp - 10;
		t1++;
	}
	return (t1<<4|(temp));
}

void convert2hex(unsigned char data, unsigned char * result)
{
	char temp;
	temp = data>>4;
	if(temp>9)
	{
		*result = temp + 55;
	}
	else
	{
		*result = temp | 0x30;
	}
	result++;
	temp = data&0x0F;
	if(temp>9)
	{
		*result = temp + 55;
	}
	else
	{
		*result = temp | 0x30;
	}
}