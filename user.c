#include <cstdio>

#include "rtos.h"
#include "LCD.h"
#include <stdio.h>
#include "Keypad.h"
#include "pwmDriver.h"

#define TIMESLICE 32000 // 2ms

void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);

uint32_t avgMotorRPM = 0;
uint32_t targetMotorRPM = 0;
// Last char needs to be the null terminator
char currentInput[5];

int32_t mutex;
int motorSpeed;
// Add mutex

void SetMotorSpeed()
{
	while (1)
	{
		OS_Wait(mutex);
		MOT34_Speed_Set(motorSpeed);
		OS_Signal(mutex);
	}
}

void InputControl()
{
	unsigned char current = ' ';
	unsigned char previous = ' ';
	char dir = ' ';
	int duty = 0;
	// Read keypad input
	while (1)
	{

		current = MatrixKeypad_Scan(&sct); // Implement Matrix scan
		if (current != previous)
		{
			switch (current)
			{ // Switch case to select speed and output dir
			case '0':
				duty = 0;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '1':
				duty = 600 * 1;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '2':
				duty = 400 * 2;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				;
				break;
			case '3':
				duty = 400 * 3;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				;
				break;
			case '4':
				duty = 400 * 4;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '5':
				duty = 400 * 5;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '6':
				duty = 400 * 6;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '7':
				duty = 400 * 7;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '8':
				duty = 400 * 8;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case '9':
				duty = 3999;
				LCD_command(0x01);
				LCD_data(dir);
				LCD_command(0x06);
				LCD_data(current);
				break;
			case 'E':
				dir = '+';
				LCD_command(0x02);
				LCD_data(dir);
				MOT34_Forward();

				break;
			case 'F':
				dir = '-';
				LCD_command(0x02);
				LCD_data(dir);
				MOT34_Reverse();

				break;
			default:
				break;
			}
			OS_Wait(mutex);
			motorSpeed = duty;
			OS_Signal(mutex);
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
	MatrixKeypad_Init();
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

	// Enable PWM on PF2
	GPIO_PORTF_AFSEL_R |= 0x4;
	GPIO_PORTF_PCTL_R &= ~(0x4 << 24);
	GPIO_PORTF_PCTL_R |= 0x4 << 24;
	GPIO_PORTF_AMSEL_R &= ~0x4;
	GPIO_PORTF_DEN_R |= 0x4;

	// Configure PWM clocking
	SYSCTL_RCC_R |= 0x1 << 20;
	SYSCTL_RCC_R |= 0x7 << 17;
	PWM1_3_CTL_R = 0;
	PWM1_3_GENA_R = 0xc8;
	PWM1_3_LOAD_R = 2499;
	PWM1_3_CMPA_R = 0;
	PWM1_3_CTL_R |= 1;
	PWM1_ENABLE_R |= 0x40;

	// TODO: Enable keypad

	// Initialize Port D switches (PD0-PD3) as inputs
	GPIO_PORTD_DIR_R &= ~0x0F; // PD3-PD0 as inputs
	GPIO_PORTD_DEN_R |= 0x0F;  // Digital enable

	OS_AddThreads(&SetMotorSpeed, &LED_Change, &Color_Add);
	OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
	return 0;			  // this never executes
}
