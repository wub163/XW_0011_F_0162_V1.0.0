#ifndef	_APP_COMM_H
#define	_APP_COMM_H

#include <stdint.h>

#define	 TRUE			1
#define	 FLASE			0

#define	 ENABLE			1
#define	 DISABLE		0

#define	 ERROR			1
#define	 NO_ERROR		0

#define	 LEVEL_HIGHT	1
#define	 LEVEL_LOW		0

#define	 MOTOR_ON		1
#define	 MOTOR_OFF		0

typedef union 
{
	uint8_t mask;
	struct 
	{
		uint8_t bit0  : 1;  
		uint8_t bit1  : 1;
		uint8_t bit2  : 1;
		uint8_t bit3  : 1;
		uint8_t bit4  : 1;
		uint8_t bit5  : 1;
		uint8_t bit6  : 1;
		uint8_t bit7  : 1;	
	}bits;
}FLAG8;

typedef union 
{
	uint16_t mask;
	struct 
	{
		uint8_t bit0  : 1;  
		uint8_t bit1  : 1;
		uint8_t bit2  : 1;
		uint8_t bit3  : 1;
		uint8_t bit4  : 1;
		uint8_t bit5  : 1;
		uint8_t bit6  : 1;
		uint8_t bit7  : 1;
		
		uint8_t bit8  : 1;
		uint8_t bit9  : 1;  
		uint8_t bit10 : 1;
		uint8_t bit11 : 1;
		uint8_t bit12 : 1;
		uint8_t bit13 : 1;
		uint8_t bit14 : 1;
		uint8_t bit15 : 1;
	}bits;
}FLAG16;

typedef union 
{
	uint32_t mask;
	struct 
	{
		uint8_t bit0  : 1;  
		uint8_t bit1  : 1;
		uint8_t bit2  : 1;
		uint8_t bit3  : 1;
		uint8_t bit4  : 1;
		uint8_t bit5  : 1;
		uint8_t bit6  : 1;
		uint8_t bit7  : 1;
		
		uint8_t bit8  : 1;
		uint8_t bit9  : 1;  
		uint8_t bit10 : 1;
		uint8_t bit11 : 1;
		uint8_t bit12 : 1;
		uint8_t bit13 : 1;
		uint8_t bit14 : 1;
		uint8_t bit15 : 1;
		
		uint8_t bit16 : 1;
		uint8_t bit17 : 1;  
		uint8_t bit18 : 1;
		uint8_t bit19 : 1;
		uint8_t bit20 : 1;
		uint8_t bit21 : 1;
		uint8_t bit22 : 1;
		uint8_t bit23 : 1;
		
		uint8_t bit24 : 1;
		uint8_t bit25 : 1;  
		uint8_t bit26 : 1;
		uint8_t bit27 : 1;
		uint8_t bit28 : 1;
		uint8_t bit29 : 1;
		uint8_t bit30 : 1;
		uint8_t bit31 : 1;
	}bits;
}FLAG32;
#endif

