#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "pwmDriver.h"
#define TIMESLICE 32000 // 2ms
#include "adcSetup.h"
#define CPU_HZ 160000000u
#define MOT34_PERIOD 100000
#define PWM_DUTY_MIN 0u
#define PWM_DUTY_MAX (MOT34_PERIOD - 1u)
#define PI_UPDATE_DIV 100u
#define ADC_REF_MV 3300u
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
#define ADC_MAX_COUNTS 255u
#define OVERSPEED_BAND_RPM 5u
#define SPEED_SENSE_FS_MV 95000u   // what Current_speed() expects at full-scale
#define SPEED_SENSE_FS_COUNTS 255u // 8-bit ADC full-scale
int32_t I = 0;
int32_t mutex = 1;
char currentInput[5] = {0};

static inline void Delay100us_Init(void)
{
    SYSCTL_RCGCTIMER_R |= (1u << 5); // enable clock to TIMER5
    (void)SYSCTL_RCGCTIMER_R;        // allow clock to start

    TIMER5_CTL_R = 0u;            // disable Timer5A during setup
    TIMER5_CFG_R = 0u;            // 32-bit timer
    TIMER5_TAMR_R = 0x02u;        // periodic mode, down-count
    TIMER5_TAILR_R = 0xFFFFFFFFu; // max reload
    TIMER5_CTL_R = 0x01u;         // enable Timer5A
}

static inline void Delay100us(void)
{
    const uint32_t ticks = (CPU_HZ / 1000u); // 100 us worth of bus cycles
    uint32_t start = TIMER5_TAR_R;           // current down-counter value
    while ((uint32_t)(start - TIMER5_TAR_R) < ticks)
    {
    }
}

void PI_Handler(void)
{
    // tighter tracking -> higher gains than before
    const int32_t kp = 4;
    const int32_t ki = 2;

    const int32_t uMin = 0;
    const int32_t uMax = (int32_t)PWM_DUTY_MAX;

    // smaller deadband so it keeps correcting down to ~±15
    const int32_t deadband = 5;

    // let it correct faster (too small here looks like "stuck" with ~100 RPM error)
    const int32_t maxStep  = 150;

    static int32_t lastU  = 0;
    static int32_t pvFilt = 0;

    if (++piCount >= PI_UPDATE_DIV)
    {
        piCount = 0;

        uint32_t sp_u, pv_u;
        OS_Wait(&mutex);
        sp_u = targetMotorRPM;
        pv_u = estMotorRPM;
        OS_Signal(&mutex);

        if (sp_u == 0)
        {
            I = 0;
            lastU = 0;
            pvFilt = 0;
            MOT34_Speed_Set(0);
            TIMER0_ICR_R = 0x01;
            return;
        }

        int32_t sp = (int32_t)sp_u;
        int32_t pv = (int32_t)pv_u;

        // faster filter (less lag) -> better convergence
        pvFilt += (pv - pvFilt) >> 2;   // alpha = 1/4

        int32_t e = sp - pvFilt;
        if (e > -deadband && e < deadband) e = 0;

        int32_t p = kp * e;

        // integrate (always), but keep I so (p+I) stays in valid PWM range
        int32_t I_cand = I + ki * e;
        int32_t iLow   = uMin - p;
        int32_t iHigh  = uMax - p;
        if (I_cand < iLow)  I_cand = iLow;
        if (I_cand > iHigh) I_cand = iHigh;

        // requested command (already in-range because of I clamp above)
        int32_t U_req = p + I_cand;
        if (U_req < uMin) U_req = uMin;
        if (U_req > uMax) U_req = uMax;

        // slew-limit the actual command
        int32_t U_act = U_req;
        int32_t diff  = U_req - lastU;
        if (diff >  maxStep) U_act = lastU + maxStep;
        if (diff < -maxStep) U_act = lastU - maxStep;
        if (U_act < uMin) U_act = uMin;
        if (U_act > uMax) U_act = uMax;

        // anti-windup vs slew: keep integrator consistent with what you ACTUALLY applied
        // (prevents slow "hunting" / staying ~100 RPM off)
        I = I_cand + (U_act - U_req);

        lastU = U_act;
        MOT34_Speed_Set((uint32_t)U_act);
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
        uint32_t mv = (avg_counts * 33000u + 127u) / 255u;

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
    int counter = 0;
    currentInput[0] = '\0';

    while (1)
    {
        Delay1ms(25);
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

                // I = 0;
                targetMotorRPM = (uint32_t)val;

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
    // MOT34_Speed_Set(2000); // ensure a valid nonzero command immediately

    OS_AddThreads(&LCDControl, &SetMotorSpeed, &InputControl);
    OS_Launch(TIMESLICE);

    return 0;
}
