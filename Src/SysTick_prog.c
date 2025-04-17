/*
 * SysTick_prog.c
 *
 *  Created on: Dec 23, 2024
 *      Author: Mai El-Shehad
 */

#include "SysTick_interface.h"
#include "RCC_interface.h"
#include "Stm32F446xx.h"
#include <stdint.h>





volatile uint32_t tick_count = 0;


void SysTick_Init(ProcessorDIV Processor) {
    // Set reload value for 1ms==1000sec tick     // Configure SysTick for 1 ms interrupts

	SysTick->LOAD = SystemCoreClock / 1000 - 1; /*SystemCoreClock=80MHZ*/

    SysTick->VAL = 0;  // Reset the current value

    // Configure the SysTick timer

    if(Processor == 1 ){
    	/*Selecting the Processor Clock (HCLK)*/
    SysTick ->CTRL |=(   1 << CLKSOURCE );  /*Use processor clock*/
    }else{
    	/*Selecting the External Clock (HCLK/8):
    	To clear the CLKSOURCE bit*/
    	SysTick ->CTRL &=~(   1 << CLKSOURCE );
    }

    SysTick ->CTRL |=(   1 << TICKINT ); /*Enable interrupt*/
	SysTick ->CTRL |=(   1 << ENABLE );  /*Enable SysTick*/
}


/**
 * @brief SysTick interrupt handler.
 */
void SysTick_Handler(void) {
    tick_count++;
}

/**
 * @brief Delay function based on SysTick.
 * @param ms: Number of milliseconds to delay.
 */
void Delay_ms(uint32_t ms) {
    uint32_t start = tick_count;
    while ((tick_count - start) < ms) {
        // Wait
    }
}

