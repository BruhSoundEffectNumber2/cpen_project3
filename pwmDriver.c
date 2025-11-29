#include "pwmDriver.h"
void MOT34_Init(uint16_t period, uint16_t duty) { 
    SYSCTL->RCGCPWM |= 0x02;  
    SYSCTL->RCGCGPIO |= 0x20; 
    SYSCTL->RCGCGPIO |= 0x02; 

    //delayMs(10); 

		SYSCTL->RCC &= ~0x00100000; 
    PWM1->_3_CTL = 0x00; //Disable PWM 
    PWM1->_3_GENA = 0xC8; 
    PWM1->_3_LOAD = period - 1; //Set period
    PWM1->_3_CMPA = duty - 1;   //Set duty cycle
    PWM1->_3_CTL |= 0x1; //Enable PWM 
    PWM1->ENABLE |= 0x40; //Enable PWM output PF2
    
    GPIOF->AFSEL |= 0x04; //Enable alternate function
    GPIOF->PCTL &= ~0x00000F00; //Clear PF2 
    GPIOF->PCTL |= 0x00000500; //Set PF2
		GPIOF->DIR |= 0x04; 
    GPIOF->DEN |= 0x04; 
    GPIOB->DEN |= 0x03; 
    GPIOB->DIR |= 0x03; 
    GPIOB->DATA &= ~0x03;
    GPIOB->DATA |= 0x02;  
	
	
		

}

void MOT34_Forward(void){ //Sets dir to forward
    GPIOB->DATA = (GPIOB->DATA & ~0x03) | 0x02;
}

void MOT34_Reverse(void){ //Sets dir reverse
    GPIOB->DATA = (GPIOB->DATA & ~0x03) | 0x01;
}
void MOT34_Speed_Set(uint16_t duty) { //Sets speed
    PWM1->_3_CMPA = duty-1;
}

