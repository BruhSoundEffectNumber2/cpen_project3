#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "pwmDriver.h"
#define TIMESLICE 32000 // 2ms
#include "adcSetup.h"
#define CPU_HZ 160000000u
#define MOT34_PERIOD   4000u
#define PWM_DUTY_MIN   0u
#define PWM_DUTY_MAX   (MOT34_PERIOD - 1u)
#define PI_UPDATE_DIV  100u

#include <time.h>
void Read_Key(void);
void Init_Keypad(void);
void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);
void Delay1ms(uint32_t n);
void PI_Timer_Init(void);
int32_t Current_speed(int32_t Avg_volt);
volatile uint32_t Key_ASCII = 0;
volatile uint32_t avgMotorRPM = 0;
volatile uint32_t targetMotorRPM = 0;
volatile uint32_t estMotorRPM = 0;
volatile uint32_t piCount = 0;
#define ADC_REF_MV       3300u
#define ADC_MAX_COUNTS   255u   
#define OVERSPEED_BAND_RPM  10u  

int32_t I = 0;
int32_t mutex = 1;
char currentInput[5] = {0};


static inline void DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable trace
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // enable cycle counter
}


static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // 100us worth of cycles
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

void PI_Handler(void)
{
    uint32_t kp = 40;
    uint32_t ki = 10;
    int32_t iLow = -4000;
    int32_t iHigh = 4000;

    if (++piCount >= PI_UPDATE_DIV)
    {
        piCount = 0;

        uint32_t sp, pv;
        OS_Wait(&mutex);
        sp = targetMotorRPM;   // RPM
        pv = estMotorRPM;      // RPM
        OS_Signal(&mutex);

        if (sp == 0u)
        {
            I = 0;
            MOT34_Speed_Set(0);
        }
        else if (pv > (sp + OVERSPEED_BAND_RPM))
        {
            I = 0;
            MOT34_Speed_Set(0);
        }
        else
        {
            int32_t e = (int32_t)sp - (int32_t)pv;
            int32_t p = (int32_t)(kp * e / 20);

            int32_t dI = (int32_t)(ki * e / 640);
            int32_t I_next = I + dI;

            if (I_next < iLow) I_next = iLow;
            else if (I_next > iHigh) I_next = iHigh;

            int32_t U_unsat = p + I_next;

            int32_t U = U_unsat;
            if (U < (int32_t)PWM_DUTY_MIN) U = (int32_t)PWM_DUTY_MIN;
            else if (U > (int32_t)PWM_DUTY_MAX) U = (int32_t)PWM_DUTY_MAX;

            if (U == U_unsat) I = I_next;

            MOT34_Speed_Set((uint32_t)U);
        }
    }

    TIMER0_ICR_R = 0x01;
}


void SetMotorSpeed(void)
{
    while (1)
    {
        uint32_t sum = 0;

        for (int i = 0; i < 100; i++)
        {
            sum += adcRead();     // 8-bit: 0..255
            Delay100us();
        }

        uint32_t mv = (sum ) / (100);
				mv = mv*10;
        int32_t rpm = Current_speed((int32_t)mv);
        if (rpm < 0) rpm = 0;

        OS_Wait(&mutex);
        avgMotorRPM = (uint32_t)rpm*5;   // display RPM
        estMotorRPM = (uint32_t)rpm;   // PI feedback RPM
        OS_Signal(&mutex);
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
			// OS_Sleep(10);
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

		OS_Sleep(1);
	}
}

int main(void)
{
	// *** LCD FIRST (before OS_Init / clock changes) ***
	OS_Init();
	OS_FIFO_Init();
	PI_Timer_Init();
	Init_LCD_Ports();
	Init_LCD();
	Init_Keypad();
	adcInit();
	DWT_DelayInit();

	// Now start the rest of the system

	MOT34_Init(4000, 0);
	MOT34_Forward();
	//MOT34_Speed_Set(2000); // ensure a valid nonzero command immediately

	OS_AddThreads(&LCDControl, &SetMotorSpeed, &InputControl);
	OS_Launch(TIMESLICE);

	return 0;
}
