// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015
// Edited by John Tadrous
// June 25, 2020

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015


 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "rtos.h"
// All TCBs
struct TCB tcbs[NUM_THREADS];
// Pointer to actively running thread
struct TCB *RunPt;
// Stacks for all threads
int32_t Stacks[NUM_THREADS][STACK_SIZE];
void PI_Handler(void);
uint32_t Mail;
int32_t Lost = 0;
int32_t Send = 0;

uint32_t PutI;
uint32_t GetI;
uint32_t Fifo[FIFO_SIZE];
int32_t CurrentSize = 0;


void PI_Timer_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x01;     // Enable Timer0 clock
    while((SYSCTL_RCGCTIMER_R & 0x01) == 0);

    TIMER0_CTL_R   &= ~0x01;        // Disable Timer0A
    TIMER0_CFG_R    = 0x00;         // 32-bit mode
    TIMER0_TAMR_R   = 0x02;         // Periodic timer, down-count
    TIMER0_TAILR_R  = 160000 - 1;   // 10 ms @ 16 MHz
    TIMER0_TAPR_R   = 0;            // No prescale

    TIMER0_ICR_R    = 0x01;         // Clear timeout flag
    TIMER0_IMR_R   |= 0x01;         // Enable timeout interrupt

    NVIC_EN0_R     |= (1 << 19);    // IRQ 19 = Timer0A

    TIMER0_CTL_R   |= 0x01;         // Enable Timer0A
}



extern void TIMER0A_Handler(void) {
    PI_Handler();
}

void OS_FIFO_Init(void)
{
  PutI = GetI = 0;
}

uint32_t OS_FIFO_Full(void)
{
  return CurrentSize == FIFO_SIZE;
}

uint32_t OS_FIFO_Empty(void)
{
  return CurrentSize == 0;
}

int32_t OS_FIFO_Put(uint32_t data)
{
  if (CurrentSize == FIFO_SIZE)
  {
    // FIFO is full
    return -1;
  }

  Fifo[PutI] = data;
  PutI = (PutI + 1) % FIFO_SIZE;
  OS_Signal(&CurrentSize);

  return 0;
}

int32_t OS_FIFO_Next(uint32_t *data)
{
  if (CurrentSize == 0)
  {
    // FIFO is empty
    return -1;
  }

  *data = Fifo[GetI];

  return 0;
}

uint32_t OS_FIFO_Get(void)
{
  uint32_t data;

  OS_Wait(&CurrentSize);

  data = Fifo[GetI];
  GetI = (GetI + 1) % FIFO_SIZE;

  return data;
}

void OS_Suspend(void)
{
  // Reset counter for fairness
  // NVIC_ST_CURRENT_R = 0;
  // Forcibly trigger an interrupt
  NVIC_INT_CTRL_R |= 0x04000000;
}

void OS_Sleep(uint32_t length)
{
  RunPt->sleep = length;
  OS_Suspend();
}

uint32_t OS_Sleep_Left(uint32_t taskId)
{
  return tcbs[taskId].sleep;
}

void OS_Wait(int32_t *s)
{
  OS_DisableInterrupts();

  (*s) -= 1;

  if ((*s) < 0)
  {
    RunPt->blocked = s;
    OS_EnableInterrupts();
    OS_Suspend();
  }

  OS_EnableInterrupts();
}

void OS_Signal(int32_t *s)
{
  struct TCB *pt;

  OS_DisableInterrupts();

  (*s) += 1;

  if ((*s) <= 0)
  {
    // Find and unblock another task
    pt = RunPt->next;

    while (pt->blocked != s)
    {
      pt = pt->next;
    }

    pt->blocked = 0;
  }

  OS_EnableInterrupts();
}

void OS_Schedule(void)
{
  // Round robin

  if (NVIC_ST_CTRL_R & 0x10000)
  {
    // A full timeslice has passed

    struct TCB *Pt = RunPt->next;

    while (Pt != RunPt)
    {
      if (Pt->sleep > 0)
      {
        Pt->sleep -= 1;
      }

      Pt = Pt->next;
    }
  }

  do
  {
    RunPt = RunPt->next;
  } while (RunPt->blocked || RunPt->sleep);
}

void SendMail(uint32_t data)
{
  Mail = data;

  if (Send)
  {
    Lost++;
  }
  else
  {
    OS_Signal(&Send);
  }
}

uint32_t RecvMail(void)
{
  OS_Wait(&Send);

  return Mail;
}

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 16 MHz clock
// input:  none
// output: none
void OS_Init(void)
{
  OS_DisableInterrupts();
  Clock_Init();                                                  // set processor clock to 16 MHz
  NVIC_ST_CTRL_R = 0;                                            // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;                                         // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0xE0000000; // priority 7
}

void SetInitialStack(int i)
{
  tcbs[i].sp = &Stacks[i][STACK_SIZE - 16]; // thread stack pointer
  Stacks[i][STACK_SIZE - 1] = 0x01000000;   // thumb bit
  Stacks[i][STACK_SIZE - 3] = 0x14141414;   // R14
  Stacks[i][STACK_SIZE - 4] = 0x12121212;   // R12
  Stacks[i][STACK_SIZE - 5] = 0x03030303;   // R3
  Stacks[i][STACK_SIZE - 6] = 0x02020202;   // R2
  Stacks[i][STACK_SIZE - 7] = 0x01010101;   // R1
  Stacks[i][STACK_SIZE - 8] = 0x00000000;   // R0
  Stacks[i][STACK_SIZE - 9] = 0x11111111;   // R11
  Stacks[i][STACK_SIZE - 10] = 0x10101010;  // R10
  Stacks[i][STACK_SIZE - 11] = 0x09090909;  // R9
  Stacks[i][STACK_SIZE - 12] = 0x08080808;  // R8
  Stacks[i][STACK_SIZE - 13] = 0x07070707;  // R7
  Stacks[i][STACK_SIZE - 14] = 0x06060606;  // R6
  Stacks[i][STACK_SIZE - 15] = 0x05050505;  // R5
  Stacks[i][STACK_SIZE - 16] = 0x04040404;  // R4
}

//******** OS_AddThread ***************
// add three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OS_AddThreads(void (*task0)(void),
                  void (*task1)(void),
                  void (*task2)(void))
{
  int32_t status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0);
  Stacks[0][STACK_SIZE - 2] = (int32_t)(task0); // PC
  SetInitialStack(1);
  Stacks[1][STACK_SIZE - 2] = (int32_t)(task1); // PC
  SetInitialStack(2);
  Stacks[2][STACK_SIZE - 2] = (int32_t)(task2); // PC
  RunPt = &tcbs[0];                             // thread 0 will run first
  EndCritical(status);
  return 1; // successful
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 60ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice)
{
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                           // start on the first task
}

void Clock_Init(void)
{
  SYSCTL_RCC_R |= 0x810;
  SYSCTL_RCC_R &= ~(0x400020);
}
