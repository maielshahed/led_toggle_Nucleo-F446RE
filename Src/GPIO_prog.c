/*
 *      @GPIO_prog.c
 *      @Created on: Dec 18, 2024
 *      @Author: Mai El Shahed
 *      @brief the GPIO main source file, including functions definitions
 */

#include <stdint.h>
#include <stddef.h>

#include "GPIO_interface.h"
#include "Stm32F446xx.h"
#include "errtype.h"


#include "GPIO_prv.h"




static GPIO_Reg_t* GPIOPORT[GPIO_PERIPHERAL_NUM] ={GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH};

/*
 * @fu GPIO_u8pininit
 * @brief the function initializes the GPIO according to the input parameters
 * @param[in]   pin_config
 * @retval

*/

uint8_t GPIO_u8pininit(const pin_configer_t* pin_config)
{
	uint8_t local_u8errorstate=OK;
	if(pin_config   != NULL ){
		if((pin_config -> port <=  PORTH) && (pin_config -> pinNum <=  PIN15)
				&& (pin_config -> mode <=  ANALOG)&& (pin_config -> speed <=  HIGH )
				&& (pin_config -> outputtype <=  OPEN_DEAIN )&& (pin_config -> pulltype <=  PULLDOWN )
				&& (pin_config -> altfunc <=  AF15 ))
		{
			/*select  GPIO port mode register (GPIOx_MODER) (: Input: output : Alternate function : Analog*/
			/*clear mode bits 0b11      */
			(GPIOPORT[pin_config -> port]->MODER) &=~(MODER_MASK << ((pin_config -> pinNum) * MODER_PIN_ACSESS ) );

			(GPIOPORT[pin_config -> port]->MODER) |= ((pin_config -> mode)<<(pin_config -> pinNum) * MODER_PIN_ACSESS );
			/*select  GPIO pull up-down---no pull set pin used  output -input -Alternate -function*/

			(GPIOPORT[pin_config -> port]->PUPDR) &=~(PUPDR_MASK << ((pin_config -> pinNum) * PUPDR_PIN_ACSESS ) );

			(GPIOPORT[pin_config -> port]->PUPDR) |= ((pin_config -> pulltype)<<(pin_config -> pinNum) * PUPDR_PIN_ACSESS );

			/*select GPIO port output speed register----> pin output or Alternate  function  */
			if((pin_config  -> mode == OUTPUT)    || (pin_config  -> mode == ALTERNATE_FUNCTION)){
				/*select output type register  0: Output push-pull - 1: Output open-drain */
				(GPIOPORT[pin_config -> port]->OTYPER) &=~(OTYPER_MASK << (pin_config -> pinNum)  );  /*clear mode bits 0b1      */

				(GPIOPORT[pin_config -> port]->OTYPER) |= ((pin_config -> outputtype)<<(pin_config -> pinNum)  );

				/*GPIO port output speed register (GPIOx_OSPEEDR) 00:Low speed---  01:Medium speed--- 10:Fast speed--- 11:High speed*/

				(GPIOPORT[pin_config -> port]->OSPEEDER) &=~(OSPEEDER_MASK << ((pin_config -> pinNum) * OSPEEDER_PIN_ACSESS ) );

				(GPIOPORT[pin_config -> port]->OSPEEDER) |= ((pin_config -> speed)<<(pin_config -> pinNum) * OSPEEDER_PIN_ACSESS );


				if(pin_config  -> mode == ALTERNATE_FUNCTION){
					/*select alternate function register   */
									uint8_t local_u8regnam=(pin_config -> pinNum) /AFR_PIN_SHIFTING ;  /* SELECT   AFRL ==0 OR  AFRH==1     /AFR_PIN_SHIFTING ==8*/
									uint8_t local_u8bitnam=(pin_config -> pinNum) %AFR_PIN_SHIFTING ;  /*NUM BIT alternate function register AFRL  OR AFRH*/
									(GPIOPORT[pin_config -> port]->AFR[local_u8regnam]) &=~(AFR_MASK << ((local_u8bitnam) * AFR_PIN_ACSESS ) );

									(GPIOPORT[pin_config -> port]->AFR[local_u8regnam]) |= ((pin_config -> altfunc)<<(local_u8bitnam) * AFR_PIN_ACSESS );


				}

			}

		}else{
			local_u8errorstate=NOK;
		}


	}else{
		local_u8errorstate=NULL_PTR_ERR ;
	}

	return local_u8errorstate;

}

/*
 * @fn GPIO_u8setpinvalue
 * @brief the function outputs certain value on an output pin
 * @param [in]   port: the port number ,get option @port_t enum
 * @param [in] pinNum: the pinNum number ,get option @pin_t enum
 * @param [in]  pinval: the output value ,get option @pinval_t enum
 * @retval local_u8errorstate

*/

uint8_t GPIO_u8setpinvalue(port_t port,pin_t pinNum,pinval_t pinval){

	uint8_t local_u8errorstate=OK;

	if(( port <=  PORTH) && (pinNum <=  PIN15))
	{

		if(pinval == PIN_LOW){
			GPIOPORT[port]->ODR &=~  (1<<pinNum);
			/* port bit set/reset register (GPIOx_BSRR)  15BIT ==1PIN ==--31 BIT==PIN15 */
			/*  GPIOPORT[port]->BSRR |= 1<<(pinNum+16) */

		}else if(pinval==PIN_HIGH){

			GPIOPORT[port]->ODR |=   (1<<pinNum);
			/* port bit set/reset register (GPIOx_BSRR)  0BIT ==1PIN ==--15 BIT==PIN15 */
						/*  GPIOPORT[port]->BSRR |= 1<<pinNum */

		}else{
			local_u8errorstate=NOK;
		}

	}else{
		local_u8errorstate=NOK;

	}


	return local_u8errorstate;

}

/*
 * @fn GPIO_u8togglepinvalue
 * @brief the function  that changes the output pin through Toggle .
 * @param[in]   port: the port number ,get option @port_t enum
 *  @param[in] pinNum: the pinNum  number ,get option @pin_t enum
 * @retval local_u8errorstate

*/
uint8_t GPIO_u8togglepinvalue(port_t port,pin_t pinNum){
	uint8_t local_u8errorstate=OK;

	if(( port <=  PORTH) && (pinNum <=  PIN15))
	{
		// Toggle the pin using the ODR
		GPIOPORT[port]->ODR ^= (1 << pinNum);

	}else{


		local_u8errorstate=NOK;
	}

	return local_u8errorstate;

}


/*
 * @fn GPIO_u8readpinvalue
 * @brief the function  that read the output pin  .
 * @param[in]   port: the port number ,get option @port_t enum
 *  @param[in] pinNum: the pinNum  number ,get option @pin_t enum
  *  @param[in]  pinval: the pointer ,get option @pinval_t enum

 * @retval local_u8errorstate

*/
uint8_t GPIO_u8readpinvalue(port_t port,pin_t pinNum,pinval_t* pinval){

	uint8_t local_u8errorstate=OK;

		if(( port <=  PORTH) && (pinNum <=  PIN15)&& (pinval != NULL))
		{
			 // Read the pin value from the IDR
			        *pinval = (GPIOPORT[port]->IDR & (1 << pinNum)) ? 1 : 0;

		}else{


			local_u8errorstate=NOK;
		}

		return local_u8errorstate;
}

