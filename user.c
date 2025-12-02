#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "pwmDriver.h"
#define TIMESLICE 32000 // 2ms
#include "adcSetup.h"
#define CPU_HZ 160000000u
#define MOT34_PERIOD   100000
#define PWM_DUTY_MIN   0u
#define PWM_DUTY_MAX   (MOT34_PERIOD - 1u)
#define PI_UPDATE_DIV  100u
#define ADC_REF_MV     3300u
#define ADC_MAX_COUNTS 255u
#define FULL_RPM 2400
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
#define ADC_MAX_COUNTS   255u   
#define OVERSPEED_BAND_RPM  5u  
#define SPEED_SENSE_FS_MV     95000u   // what Current_speed() expects at full-scale
#define SPEED_SENSE_FS_COUNTS 255u     // 8-bit ADC full-scale
int32_t I = 0;
int32_t mutex = 1;
char currentInput[5] = {0};


static inline void Delay100us_Init(void)
{
    SYSCTL_RCGCTIMER_R |= (1u << 5);   // enable clock to TIMER5
    (void)SYSCTL_RCGCTIMER_R;          // allow clock to start

    TIMER5_CTL_R = 0u;                // disable Timer5A during setup
    TIMER5_CFG_R = 0u;                // 32-bit timer
    TIMER5_TAMR_R = 0x02u;            // periodic mode, down-count
    TIMER5_TAILR_R = 0xFFFFFFFFu;     // max reload
    TIMER5_CTL_R = 0x01u;             // enable Timer5A
}

static inline void Delay100us(void)
{
    const uint32_t ticks = (CPU_HZ / 1000u);  // 100 us worth of bus cycles
    uint32_t start = TIMER5_TAR_R;             // current down-counter value
    while ((uint32_t)(start - TIMER5_TAR_R) < ticks) { }
}

void PI_Handler(void)
{
    // ----- GAINS -----
    const int32_t kp = 1;
    const int32_t ki = 3;

    // ----- INTEGRAL LIMITS -----
    const int32_t iLow  = -3000;
    const int32_t iHigh =  3000;

    // ----- ERROR FILTER -----
    static int32_t eFilt = 0;
    const int32_t FILTER_N = 4;

    // ----- PWM OUTPUT MEMORY -----
    static int32_t lastU = 0;

    // ----- MAX MOTOR RPM (required for scaling RPM -> PWM duty) -----
    

    if (++piCount >= PI_UPDATE_DIV)
    {
        piCount = 0;

        uint32_t sp, pv;
        OS_Wait(&mutex);
        sp = targetMotorRPM;
        pv = estMotorRPM;
        OS_Signal(&mutex);

        // Target = 0 → stop everything cleanly
        if (sp == 0)
        {
            I = 0;
            lastU = 0;
            MOT34_Speed_Set(0);
            TIMER0_ICR_R = 0x01;
            return;
        }

        // Compute error
        int32_t e = (int32_t)sp - (int32_t)pv;

        // Small deadband
        if (e > -3 && e < 3) e = 0;

        // Filter error
        eFilt = ((FILTER_N - 1) * eFilt + e) / FILTER_N;

        // P term
        int32_t p = kp * eFilt;

        // I term (with limiting)
        int32_t I_next = I + ki * eFilt;
        if (I_next < iLow)  I_next = iLow;
        if (I_next > iHigh) I_next = iHigh;

        // RAW PI output in RPM units
        int32_t U_rpm = p + I_next;

        // ----- SCALE TO PWM DUTY RANGE -----
        int32_t U_unsat = (U_rpm * PWM_DUTY_MAX) / FULL_RPM;

        // Hard clamp to PWM boundaries
        if (U_unsat < PWM_DUTY_MIN) U_unsat = PWM_DUTY_MIN;
        if (U_unsat > PWM_DUTY_MAX) U_unsat = PWM_DUTY_MAX;

        // ----- DYNAMIC SLEW RATE LIMITER -----
        int32_t absE = (eFilt >= 0 ? eFilt : -eFilt);
        int32_t maxStep;

        if      (absE > 500) maxStep = 120;
        else if (absE > 200) maxStep = 60;
        else if (absE > 100) maxStep = 30;
        else if (absE > 50)  maxStep = 10;
        else if (absE > 20)  maxStep = 5;
        else                 maxStep = 1;

        int32_t U = U_unsat;
        int32_t diff = U - lastU;

        if (diff >  maxStep) U = lastU + maxStep;
        if (diff < -maxStep) U = lastU - maxStep;

        lastU = U;

        // Accept integrator only if no PWM saturation
        if (U_unsat == (int32_t)((U_rpm * PWM_DUTY_MAX) / FULL_RPM))
            I = I_next;

        MOT34_Speed_Set((uint32_t)U);
    }

    TIMER0_ICR_R = 0x01;
}





void SetMotorSpeed(void)
{
    static int32_t filt_rpm = 0;

    while (1)
    {
        uint32_t sum = 0;

        for (int i = 0; i < 100; i++)
        {
            sum += (uint32_t)adcRead();   // 8-bit: 0..255
            Delay100us();
        }

        uint32_t avg_counts = (sum + 50u) / 100u; // rounded average (0..255)

        // Map 0..255 counts -> 0..95000 mV (matches Current_speed() expected input range)
        uint32_t mv = (avg_counts * 33000u + 127u) / 255u;



        int32_t rpm = Current_speed((int32_t)mv);
        if (rpm < 0) rpm = 0;

        // Simple LPF to stop 0/9999 bouncing (IIR: y += (x-y)/4)
        filt_rpm += (rpm - filt_rpm) >> 2;

        OS_Wait(&mutex);
        avgMotorRPM = (uint32_t)filt_rpm;
        estMotorRPM = (uint32_t)filt_rpm;
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
	Delay100us_Init();

	// Now start the rest of the system

	MOT34_Init(100000, 0);
	MOT34_Forward();
	//MOT34_Speed_Set(2000); // ensure a valid nonzero command immediately

	OS_AddThreads(&LCDControl, &SetMotorSpeed, &InputControl);
	OS_Launch(TIMESLICE);

	return 0;
}
