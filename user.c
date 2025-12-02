#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "pwmDriver.h"
#define TIMESLICE 32000 // 2ms
#include "adcSetup.h"
#define CPU_HZ 160000000u
#define MOT34_PERIOD 4000u
#define PWM_DUTY_MIN 0u
#define PWM_DUTY_MAX (MOT34_PERIOD - 1u)
#define PI_UPDATE_DIV 100u
#define ADC_REF_MV 3300u
#define ADC_MAX_COUNTS 255u

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
#define ADC_REF_MV 10000u
#define ADC_MAX_COUNTS 255u
#define OVERSPEED_BAND_RPM 5u
#define SPEED_SENSE_FS_MV 95000u   // what Current_speed() expects at full-scale
#define SPEED_SENSE_FS_COUNTS 255u // 8-bit ADC full-scale
int32_t I = 0;
int32_t mutex = 1;
char currentInput[5] = {0};

static inline void DWT_DelayInit(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable trace
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable cycle counter
}

static inline void Delay100us(void)
{
	const uint32_t cycles = (CPU_HZ / 10000u); // 100us worth of cycles
	uint32_t start = DWT->CYCCNT;
	while ((uint32_t)(DWT->CYCCNT - start) < cycles)
	{
	}
}

void PI_Handler(void)
{
	int32_t kp = 1;
	int32_t ki = 2;
	int32_t kd = 2; // <-- ADDED (tune this)

	int32_t iLow = -3000;
	int32_t iHigh = 3000;

	static int32_t lastU = 0;

	// filtered error
	static int32_t eFilt = 0;
	const int32_t FILTER_N = 4;

	// derivative on measurement (damping, less noise than d(error) when setpoint changes)
	static int32_t pvPrev = 0;	// <-- ADDED
	static int32_t dFilt = 0;	// <-- ADDED
	const int32_t D_FILT_N = 4; // <-- ADDED

	if (++piCount >= PI_UPDATE_DIV)
	{
		piCount = 0;

		uint32_t sp, pv;
		OS_Wait(&mutex);
		sp = targetMotorRPM;
		pv = estMotorRPM;
		OS_Signal(&mutex);

		if (sp == 0)
		{
			I = 0;
			lastU = 0;
			pvPrev = (int32_t)pv; // <-- ADDED
			dFilt = 0;			  // <-- ADDED
			MOT34_Speed_Set(0);
			TIMER0_ICR_R = 0x01;
			return;
		}

		int32_t e = (int32_t)sp - (int32_t)pv;

		// tiny deadband
		if (e > -3 && e < 3)
			e = 0;

		// faster filter
		eFilt = ((FILTER_N - 1) * eFilt + e) / FILTER_N;

		// PI components
		int32_t p = kp * eFilt;

		int32_t I_next = I + ki * eFilt;
		if (I_next < iLow)
			I_next = iLow;
		if (I_next > iHigh)
			I_next = iHigh;

		// --- ADDED: D term (derivative on pv), filtered ---
		int32_t dpv = (int32_t)pv - pvPrev;
		pvPrev = (int32_t)pv;

		dFilt = ((D_FILT_N - 1) * dFilt + dpv) / D_FILT_N;

		// If pv is rising fast, subtract to add damping (slows the *change*, not the target)
		int32_t d = -kd * dFilt;
		// -----------------------------------------------

		int32_t U_unsat = p + I_next + d; // <-- CHANGED (added d)

		// clamp to PWM range
		int32_t U_clamped = U_unsat;
		if (U_clamped < PWM_DUTY_MIN)
			U_clamped = PWM_DUTY_MIN;
		if (U_clamped > PWM_DUTY_MAX)
			U_clamped = PWM_DUTY_MAX;

		// DYNAMIC STEP SIZE (FAST → SLOW → PRECISION)
		int32_t absE = (e >= 0 ? e : -e);
		int32_t maxStep;

		if (absE > 500)
			maxStep = 120;
		else if (absE > 200)
			maxStep = 60;
		else if (absE > 100)
			maxStep = 30;
		else if (absE > 50)
			maxStep = 10;
		else if (absE > 20)
			maxStep = 5;
		else
			maxStep = 1;

		// apply slew limit
		int32_t U = U_clamped;
		int32_t diff = U - lastU;
		if (diff > maxStep)
			U = lastU + maxStep;
		if (diff < -maxStep)
			U = lastU - maxStep;

		lastU = U;

		// <-- CHANGED: only block integral on *PWM saturation*, not on slew limiting
		if (U_clamped == U_unsat)
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
			sum += (uint32_t)adcRead(); // 8-bit: 0..255
			Delay100us();
		}

		uint32_t avg_counts = (sum + 50u) / 100u; // rounded average (0..255)

		// Map 0..255 counts -> 0..95000 mV (matches Current_speed() expected input range)
		uint32_t mv = (avg_counts * ADC_REF_MV + 127) / 255;

		int32_t rpm = Current_speed((int32_t)mv);
		if (rpm < 0)
			rpm = 0;

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
	int pressed = 0;
	int counter = 0;
	currentInput[0] = '\0';

	while (1)
	{
		Delay1ms(50);
		Read_Key();
		uint32_t test = Key_ASCII;
		unsigned char current = (unsigned char)(Key_ASCII & 0xFF);
		if (current != 0x00 && !pressed)
		{
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
		else if (pressed)
		{
			pressed = 0;
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
	// MOT34_Speed_Set(2000); // ensure a valid nonzero command immediately

	OS_AddThreads(&LCDControl, &SetMotorSpeed, &InputControl);
	OS_Launch(TIMESLICE);

	return 0;
}
