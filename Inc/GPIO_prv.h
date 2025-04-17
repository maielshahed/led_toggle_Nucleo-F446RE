/*
 * GPIO_prv.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Mai El Shahed
 */

#ifndef GPIO_PRV_H_
#define GPIO_PRV_H_

#define GPIO_PERIPHERAL_NUM 8u

#define MODER_MASK         0b11
#define MODER_PIN_ACSESS   2u

#define PUPDR_MASK         0b11   /*PUPDR[1:0] 2BIT 0b11*/
#define PUPDR_PIN_ACSESS   2u    /* pin  porta pin 5 ----2 bit ---PUPDR start pin10-*/


#define OSPEEDER_MASK         0b11  /*GPIO port output speed register*/
#define OSPEEDER_PIN_ACSESS   2u

#define OTYPER_MASK         0b1    //GPIO port output type register


/* alternate function low register*/
#define AFR_MASK              0b1111
#define AFR_PIN_ACSESS         4U
#define AFR_PIN_SHIFTING       8U



#endif /* GPIO_PRV_H_ */
