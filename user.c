#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "pwmDriver.h"
#define TIMESLICE 32000 // 2ms
#include "adcSetup.h"
#include <time.h>
void Read_Key(void);
void Init_Keypad(void);
void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);
void Delay1ms(uint32_t n);

volatile uint32_t Key_ASCII = 0;
volatile uint32_t avgMotorRPM = 0;
volatile uint32_t targetMotorRPM = 0;
volatile uint32_t estMotorRPM = 0;
volatile uint32_t piCount = 0;

int32_t I = 0;
int32_t mutex = 1;			
char currentInput[5] = {0}; 


void PI_Handler(void)
{
	uint32_t kp = 105;
	uint32_t ki = 101;
	int32_t iLow = -500;
	int32_t iHigh = 4000;

	if (++piCount == 4000)
	{
		piCount = 0;
		int32_t e = (int32_t)targetMotorRPM - (int32_t)estMotorRPM;
		int32_t p = (int32_t)(kp * e / 20);
		I += (int32_t)(ki * e / 640);

		if (I < iLow)
			I = iLow;
		else if (I > iHigh)
			I = iHigh;

		int32_t U = p + I;

		if (U < 100)
			U = 100;
		else if (U > 19900)
			U = 19900;

		MOT34_Speed_Set((uint32_t)U);
	}

	TIMER0_ICR_R = 0x01; 
}

void SetMotorSpeed(void)
{
	while (1)
	{
		uint32_t cmd;

		OS_Wait(&mutex);
		cmd = targetMotorRPM;
		OS_Signal(&mutex);

		MOT34_Speed_Set(cmd);

		OS_Sleep(5); // don’t spam the driver
	}
}

void InputControl(void)
{
	int counter = 0;
	currentInput[0] = '\0';

	while (1)
	{
		Delay1ms(50);
		Read_Key();
		uint32_t test = Key_ASCII;
		unsigned char current = (unsigned char)(Key_ASCII & 0xFF);
		if (current != 0x00)
		{
			Key_ASCII = 0; // consume

			if ((current >= '0' && current <= '9') && counter < 4)
			{
				currentInput[counter++] = (char)current;
				currentInput[counter] = '\0';
			}
			else if (current == '#')
			{
				int val = atoi(currentInput);

				OS_Wait(&mutex);
				targetMotorRPM = (uint32_t)val;
				OS_Signal(&mutex);

				counter = 0;
				currentInput[0] = '\0';
			}
			else if (current == 'C')
			{
				counter = 0;
				currentInput[0] = '\0';
			}
			//OS_Sleep(10);
		}

		
	}
}

void LCDControl(void)
{
	char line1[17];
	char line2[17];
	uint32_t t, a;
	while (1)
	{
		
		
		

		OS_Wait(&mutex);
		t = targetMotorRPM;
		a = avgMotorRPM;
		OS_Signal(&mutex);

		

		snprintf(line1, sizeof(line1), "Input RPM:%-4.4s", currentInput);
		snprintf(line2, sizeof(line2), "T:%04lu C:%04lu",
				 (unsigned long)((t > 9999u) ? 9999u : t),
				 (unsigned long)((a > 9999u) ? 9999u : a));

		Set_Position(0x00);
		Display_Msg(line1);
		Set_Position(0x40);
		Display_Msg(line2);

		// OS_Sleep(1);
	}
}

int main(void)
{
	// *** LCD FIRST (before OS_Init / clock changes) ***
	OS_Init();
	OS_FIFO_Init();
	Init_LCD_Ports();
	Init_LCD();
	Init_Keypad();
	adcInit();
	
	// Now start the rest of the system

	MOT34_Init(4000, 0);
	MOT34_Forward();

	OS_AddThreads(&LCDControl, &SetMotorSpeed, &InputControl);
	OS_Launch(TIMESLICE);

	return 0;
}
