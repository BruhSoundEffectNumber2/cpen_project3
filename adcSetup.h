#ifndef ADCSETUP_H
#define ADCSETUP_H

#include "tm4c123gh6pm.h"
#include <stdint.h>


void adcInit(void);
uint32_t adcRead(void);

#endif
