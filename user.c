#include <cstdio>

#include "rtos.h"
#include "LCD.h"
#include <stdio.h>
#include "Keypad.h"
#include "pwmDriver.h"
#include <stdlib.h>
#define TIMESLICE 32000 // 2ms

void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);

uint32_t avgMotorRPM = 0;
uint32_t targetMotorRPM = 0;
uint32_t estMotorRPM = 0;
// Incrimented every 0.1ms for PI controller timing
uint32_t piCount = 0;

// PI I term
int32_t I = 0;

// Last char needs to be the null terminator
char currentInput[5];

// Add mutex

void PI_Handler(void)
{
	uint32_t kp = 105;	  // Proportional coefficient
	uint32_t ki = 101;	  // Integral coefficient
	int32_t iLow = -500;  // Integral minimum
	int32_t iHigh = 4000; // Integral maximum

	// Every 0.4 ms
	if (++piCount == 4000)
	{
		piCount = 0;

		// Basic PI loop
		int32_t e = targetMotorRPM - estMotorRPM;
		int32_t p = kp * e / 20;
		I += ki * e / 640;

		// Bound I term
		if (I < iLow)
			I = iLow;
		else if (I > iHigh)
			I = iHigh;

		int32_t U = p + I;

		// Constrain actuator output
		if (U < 100)
			U = 100;
		else if (U > 19900)
			U = 19900;

		MOT34_Speed_Set(U);
	}

	TIMER0_ICR_R = 0x01;
}

void SetMotorSpeed()
{
	while (1)
	{
		// OS_Wait(mutex);
		// MOT34_Speed_Set(targetMotorRPM);
		// OS_Signal(mutex);
	}
}

void InputControl()
{
	unsigned char current = ' ';
	unsigned char previous = ' ';
	char dir = ' ';
	int duty = 0;
	int counter = 0;
	// Read keypad input
	while (1)
	{

		current = Read_Key(&sct); // Implement Matrix scan
		if (current != 0x00)
		{
			if ((current <= '9' || current >= '0') && counter < 4)
			{

				currentInput[current] = current;
				counter++;
			}
			else if (current == '#')
			{
				counter = 0;
				duty = atoi(currentInput);
				OS_Wait(mutex);
				targetMotorRPM = duty;
				OS_Signal(mutex);
			}
			else if(current== 'C')
			{
				for(int i=0 i<4; i++)
				{
					currentInput[i]='\0';
				}
				counter=0;
			}
		}
	}
}

void LCDControl()
{
	char rpmStr[5];

	while (1)
	{
		// Clear LCD
		Set_Position(0x00);
		Display_Msg("                ");
		Set_Position(0x40);
		Display_Msg("                ");

		// Top
		Set_Position(0x00);
		Display_Msg("Input RPM: ");
		Display_Msg(currentInput);

		// Bottom
		Set_Position(0x40);
		sprintf(rpmStr, "T: %04d", targetMotorRPM);
		Display_Msg(rpmStr);
		sprintf(rpmStr, "C: %04d", avgMotorRPM);
		Display_Msg(rpmStr);

		// 4hz
		OS_Sleep(125);
	}
}

int main(void)
{
	uint16_t period = 4000;
	uint16_t duty = 0; // Start at zero
	MOT34_Init(period, duty);
	LCD_init();

	OS_Init(); // initialize, disable interrupts, 16 MHz
	OS_FIFO_Init();

	// Enable all GPIO Ports and wait for them to stabilize
	SYSCTL_RCGCGPIO_R = 0x3F;
	// Enable PWM Module 1
	SYSCTL_RCGCPWM_R |= 0x2;

	while ((SYSCTL_RCGCGPIO_R & 0x3F) == 0)
	{
	}

	Init_LCD_Ports();
	Init_LCD();

	// // Enable PWM on PF2
	// GPIO_PORTF_AFSEL_R |= 0x4;
	// GPIO_PORTF_PCTL_R &= ~(0x4 << 24);
	// GPIO_PORTF_PCTL_R |= 0x4 << 24;
	// GPIO_PORTF_AMSEL_R &= ~0x4;
	// GPIO_PORTF_DEN_R |= 0x4;

	// // Configure PWM clocking
	// SYSCTL_RCC_R |= 0x1 << 20;
	// SYSCTL_RCC_R |= 0x7 << 17;
	// PWM1_3_CTL_R = 0;
	// PWM1_3_GENA_R = 0xc8;
	// PWM1_3_LOAD_R = 2499;
	// PWM1_3_CMPA_R = 0;
	// PWM1_3_CTL_R |= 1;
	// PWM1_ENABLE_R |= 0x40;

	// // TODO: Enable keypad

	// // Initialize Port D switches (PD0-PD3) as inputs
	// GPIO_PORTD_DIR_R &= ~0x0F; // PD3-PD0 as inputs
	// GPIO_PORTD_DEN_R |= 0x0F;  // Digital enable

	OS_AddThreads(&SetMotorSpeed, &LED_Change, &Color_Add);
	OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
	return 0;			  // this never executes
}
