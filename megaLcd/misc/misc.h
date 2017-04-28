/*
 * misc.h
 *
 * Created: 12/11/2016 12:13:14 AM
 *  Author: ganzo
 */ 


// Compare in1 and in2 using len
unsigned char databyte_cmp(unsigned char *in1, unsigned char *in2, unsigned char len);
// search if in1 is found in in2
unsigned char cmd_cmp(const char *in1, char *in2);
// convert 8 bit unsigned to BCD
unsigned char convert2bcd(unsigned char temp);

// convert single 8bit to hex string, appended null at the end.
void convert2hex(unsigned char data, unsigned char * result);