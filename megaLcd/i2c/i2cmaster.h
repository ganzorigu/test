/*
 * i2cmaster.h
 *
 * Created: 11/22/2016 11:30:24
 *  Author: lab
 */ 

//#define F_CPU 16000000UL
#define SCL_FREQ 100000L //; 100kHz
#define TWBR_VAL (unsigned char)(((F_CPU / SCL_FREQ)-16)/2)

void i2c_Init();
unsigned char i2c_Start();
unsigned char i2c_RepStart();
unsigned char i2c_AddWrite(unsigned char SlaveAddr);
unsigned char i2c_DataWrite(unsigned char i2cTxData);
unsigned char i2c_AddrRead(unsigned char SlaveAddr);
unsigned char i2c_DataReadAck();
unsigned char i2c_DataReadNAck();
void i2c_Stop();