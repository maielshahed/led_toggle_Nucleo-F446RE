/*
 * SysTick_interface.h
 *
 *  Created on: Dec 23, 2024
 *      Author: Mai El-Shehad
 */

#ifndef SYSTICK_INTERFACE_H_
#define SYSTICK_INTERFACE_H_

#include <stdint.h>

enum {
	ENABLE=0,   /*0 Counter disabled - 1 Enable SysTick  bit position */
	TICKINT,   /*0:disabled SysTick exception request-- 1: Enable SysTick Interrupt  bit position   */
	CLKSOURCE,  /*Clock Source  0: AHB/8  --  1: Processor clock (AHB)  bit position  */
	COUNTFLAG  /* Count Flag  bit position */
};
/***************************************************
 * @ProcessorDIV enum
 *
 *
 */
typedef enum{
	ProcessorAHBDIV8, /*Selecting the External Clock (HCLK/8):*/

	ProcessorAHB /*Selecting the Processor Clock (HCLK)*/
}ProcessorDIV;


/*Configure SysTick for 1 ms interrupts*/
void SysTick_Init(ProcessorDIV Processor);
/*SysTick interrupt handler.*/
void SysTick_Handler(void);

/*Delay function based on SysTick.*/
void Delay_ms(uint32_t ms);

#endif /* SYSTICK_INTERFACE_H_ */
