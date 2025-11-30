#include "adcSetup.h"
#include <stdint.h>

void adcInit(void)
{
    SYSCTL->RCGCGPIO |= (1<<5) | (1<<4) | (1<<0);

    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR   |= 0x01;
    GPIOF->LOCK = 0x00;
    GPIOF->DIR &= ~( (1<<0) | (1<<1) | (1<<3) | (1<<4) );
    GPIOF->DEN |=   (1<<0) | (1<<1) | (1<<3) | (1<<4);

    GPIOE->DIR &= ~( (1<<1) | (1<<2) | (1<<3) | (1<<4) );
    GPIOE->DEN |=   (1<<1) | (1<<2) | (1<<3) | (1<<4);

    GPIOA->DIR |=  (1<<7);   // R/C output
    GPIOA->DIR &= ~(1<<6);   // BUSY input
    GPIOA->DEN |=  (1<<7) | (1<<6);

    // R/C idle high
    GPIOA->DATA |= (1<<7);
}

uint32_t adcRead(void)
{
    GPIOA->DATA |=  (1<<7);
    GPIOA->DATA &= ~(1<<7);
    GPIOA->DATA |=  (1<<7);
    while (GPIOA->DATA & (1<<6));
    while (!(GPIOA->DATA & (1<<6)));

    uint32_t d0 = (GPIOF->DATA & (1<<0)) >> 0;   // PF0 = D0
    uint32_t d1 = (GPIOF->DATA & (1<<1)) >> 1;   // PF1 = D1
    uint32_t d2 = (GPIOE->DATA & (1<<1)) >> 1;   // PE1 = D2
    uint32_t d3 = (GPIOF->DATA & (1<<3)) >> 3;   // PF3 = D3
    uint32_t d4 = (GPIOF->DATA & (1<<4)) >> 4;   // PF4 = D4
    uint32_t d5 = (GPIOE->DATA & (1<<2)) >> 2;   // PE2 = D5
    uint32_t d6 = (GPIOE->DATA & (1<<3)) >> 3;   // PE3 = D6
    uint32_t d7 = (GPIOE->DATA & (1<<4)) >> 4;   // PE4 = D7
    return (d7<<7) | (d6<<6) | (d5<<5) | (d4<<4) |
           (d3<<3) | (d2<<2) | (d1<<1) | d0;
}
