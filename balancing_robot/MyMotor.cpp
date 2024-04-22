#include "Arduino.h"
#include "MyMotor.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


void Motor_Init(void)
{
	//DDC3: in1, DDC2:in2, DDC0:in3, DDC1:in4
	DDRC |= (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3);
	PORTC |= (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3);
}

void Right_Motor(int speed)
{
	if(speed>=0)
	{
		analogWrite(6, speed);
		analogWrite(5, 0);
	}
	
	if(speed<0)
	{
		analogWrite(6, 0);
		analogWrite(5, (-1)*speed);
	}
}

void Left_Motor(int speed)
{
	if(speed>=0)
	{
		analogWrite(10, speed);
		analogWrite(9, 0);
	}
	
	if(speed<0)
	{
		analogWrite(10, 0);
		analogWrite(9, (-1)*speed);
	}
}

void Back_N_Foward(int speed, int deadzone)
{
	if(speed==0)
	{
		Right_Motor(0);
		Left_Motor(0);
	}
	if(speed>0)
	{
		speed=MAX(speed, deadzone);
		Right_Motor(speed);
		Left_Motor(speed);
	}
	if(speed<0)
	{
		speed=MIN(speed, (-1)*deadzone);
		Right_Motor(speed);
		Left_Motor(speed);	
	}
}

void Left_N_Right(int speed, int deadzone)
{
	if(speed==0)
	{
		Right_Motor(0);
		Left_Motor(0);
	}
	if(speed>0)
	{
		speed=MAX(speed, deadzone);
		Right_Motor(speed);
		Left_Motor(-speed);
	}
	if(speed<0)
	{
		speed=MIN(speed, (-1)*deadzone);
		Right_Motor(speed);
		Left_Motor((-1)*speed);
	}
}
