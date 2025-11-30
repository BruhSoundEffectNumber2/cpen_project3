#include "adcSetup.h"
#include <stdint.h>

void adcInit(void)
{
    SYSCTL->RCGCGPIO |= (1<<1);
    GPIOB->DIR &= ~( (1<<7) | (1<<6) | (1<<5) | (1<<4) ); // inputs
    GPIOB->DEN |=   ( (1<<7) | (1<<6) | (1<<5) | (1<<4) );
    // PB3 = R/C  (output)
    // PB2 = BUSY (input)
    GPIOB->DIR |=  (1<<3);   // R/C output
    GPIOB->DIR &= ~(1<<2);   // BUSY input
    GPIOB->DEN |=  (1<<3) | (1<<2);
    GPIOB->DATA |= (1<<3);
}

uint32_t adcRead(void)
{
    GPIOB->DATA |=  (1<<3);   // HIGH
    GPIOB->DATA &= ~(1<<3);   // LOW start conversion
    GPIOB->DATA |=  (1<<3);   // HIGH enable output

    while (GPIOB->DATA & (1<<2));     

    while (!(GPIOB->DATA & (1<<2)));  
    uint32_t bit3 = (GPIOB->DATA & (1<<7)) >> 7; 
    uint32_t bit2 = (GPIOB->DATA & (1<<6)) >> 6;
    uint32_t bit1 = (GPIOB->DATA & (1<<5)) >> 5; 
    uint32_t bit0 = (GPIOB->DATA & (1<<4)) >> 4;

    return (bit3<<3) | (bit2<<2) | (bit1<<1) | bit0;
}
