/*
 * i2cmaster.c
 *
 * Created: 11/22/2016 10:53:47
 *  Author: lab
 */ 

#include <util/twi.h>
#include "i2cmaster.h"

void i2c_Init()
{
	TWSR = 0x00;	// set the status reg to 0x00
	TWDR = 0xFF;	
	TWBR = TWBR_VAL;		// set the bit rate register
	TWCR = (1<<TWEN);	
}

unsigned char i2c_Start()
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_START)
	{
		return 0;	
	}
	return 1;	
}

unsigned char i2c_RepStart()
{
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while (!(TWCR & (1<<TWINT)));
		if ((TWSR & 0xF8) != TW_REP_START)
		{
			return 0;
		}
		return 1;
}

unsigned char i2c_AddWrite(unsigned char SlaveAddr)
{
	TWDR = SlaveAddr;			 // load slave address
	TWCR = (1<<TWINT)|(1<<TWEN); // enable twint, twen
	while(!(TWCR&(1<<TWINT)));	 // Wait until twint flag is set
	if ((TWSR & 0xF8) != TW_MT_SLA_ACK)
	{
		return 0;
	}
	return 1;	
}

unsigned char i2c_DataWrite(unsigned char i2cTxData)
{
	TWDR = i2cTxData;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR&(1<<TWINT)));	 // Wait until twint flag is set
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
	{
		return 0;
	}
	return 1;
}

unsigned char i2c_AddrRead(unsigned char SlaveAddr)
{
	TWDR = SlaveAddr | 0x01;  // SLA+R
	TWCR = (1<<TWINT)|(1<<TWEN);	// enable twint, twen
	while(!(TWCR&(1<<TWINT)));	 // Wait until twint flag is set
	if ((TWSR & 0xF8) != TW_MR_SLA_ACK)
	{
		return 0;
	}
	return 1;
}

unsigned char i2c_DataReadAck()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR&(1<<TWINT)));	 // Wait until twint flag is set
	if ((TWSR & 0xF8) != TW_MR_DATA_ACK)
	{
		return 0;
	}
	// need to read TWDR
	return 1;
}

unsigned char i2c_DataReadNAck()
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR&(1<<TWINT)));	 // Wait until twint flag is set
	if ((TWSR & 0xF8) != TW_MR_DATA_NACK)
	{
		return 0;
	}
	// need to read TWDR
	return 1;
}

void i2c_Stop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	// stop condition
	while(TWCR&(1<<TWSTO));					// TWSTO will clear after stop condition is over.	
}
