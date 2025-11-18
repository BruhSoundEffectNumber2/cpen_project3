#include "rtos.h"

#define TIMESLICE 32000 // 2ms

void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);

void Set_Motor_Duty(uint8_t percent)
{
	if (percent > 100)
	{
		percent = 100;
	}

	PWM1_3_CMPA_R = percent * 2500 - 1;
}

void LCD_Display()
{
	while (1)
	{
	}
}

void LED_Change()
{
}

void Color_Add()
{
}

int main(void)
{
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

	OS_AddThreads(&LCD_Display, &LED_Change, &Color_Add);
	OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
	return 0;			  // this never executes
}
