/*
 * Stm32F446xx.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Mai El Shahed
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_


/*******************memories base addresses**********/
#define FLASH_BASE_ADDRESS 0x08000000UL
#define SRAM_BASE_ADDRESS 0x02000000UL
#define ROM_BASE_ADDRESS 0x1FFF0000UL

/*******AHB1 peripheral base addresses **************/
#define GPIOA_BASE_ADDRESS 0x40020000U
#define GPIOB_BASE_ADDRESS 0x40020400U
#define GPIOC_BASE_ADDRESS 0x40020800U
#define GPIOD_BASE_ADDRESS 0x40020C00U
#define GPIOE_BASE_ADDRESS 0x40021000U
#define GPIOF_BASE_ADDRESS 0x40021400U
#define GPIOG_BASE_ADDRESS 0x40021800U
#define GPIOH_BASE_ADDRESS 0x40021C00U


#define RCC_BASE_ADDRESS 0x40023800U

#define SysTick_BASE_ADDRESS 0xE000E010U   /*System Timer, see The system timer, SysTick*/
/*******AHB2 peripheral base addresses **************/

/*******AHB3 peripheral base addresses **************/

/*******APB1 peripheral base addresses **************/

/*******APB2 peripheral base addresses **************/

/*******GPIO registers Definition Structure *****GPIO register map*********/
typedef struct{
	volatile uint32_t MODER;          /* mode register (00: Input (reset state)  01: General purpose output mode---10: Alternate function mode---11: Analog mode*/
	volatile uint32_t OTYPER;         /*output type register  0: Output push-pull -1: Output open-drain Address offset: 0x04*/
	volatile uint32_t OSPEEDER;        /*Address offset: 0x08 output speed register*/
	volatile uint32_t PUPDR;   /*Address offset: 0x0C  pull-up/pull-down register*/
	volatile uint32_t IDR;   /*Address offset: 0x10  input data register*/
	volatile uint32_t ODR;    /*Address offset: 0x014  output data register*/
	volatile uint32_t BSRR;   /*Address offset: 0x18   bit set/reset register*/
	volatile uint32_t LCKR;   /*configuration lock register*/
    volatile uint32_t AFR[2];   /*alternate function low&high register*/


	//volatile uint32_t AFRL; /*alternate function low register*/
	//volatile uint32_t AFRH; /*alternate function high register*/

}GPIO_Reg_t;


/*******RCC registers Definition Structure **************/

typedef struct{
	volatile uint32_t CR;          // Clock control register address Offset
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;       // Clock configuration register (RCC_CFGR)Address offset: 0x08
	volatile uint32_t CIR;        //Address offset: 0x0c
	volatile uint32_t AHB1RSTR;   //Address offset: 0x010
	volatile uint32_t AHB2RSTR;   //Address offset: 0x014
	volatile uint32_t AHB3RSTR;   //Address offset: 0x018

	volatile uint32_t Reserved;   //Address offset: Reserved Reset value

	volatile uint32_t APB1RSTR;    //
	volatile uint32_t APB2RSTR;

	volatile uint32_t Reserved2[2];   //Address offset: Reserved Reset value

	volatile uint32_t AHB1ENR; //write add 0x30
	volatile uint32_t AHB2ENR;   //Address offset: 0x34
	volatile uint32_t AHB3ENR;   //Address offset: 0x38


	volatile uint32_t Reserved3;   //Address offset: Reserved Reset value

	volatile uint32_t APB1ENR;  //Address offset: 0x40
	volatile uint32_t APB2ENR; //Address offset: 0x44

        volatile uint32_t Reserved4[2];

        volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;

        volatile uint32_t Reserved5;

	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;

	volatile uint32_t Reserved6[2];   //Address offset: Reserved Reset value-8 byte unsigned integer

	volatile uint32_t BDCR;
	volatile uint32_t CSR;

	volatile uint32_t Reserved7[2];   //Address offset: Reserved Reset value-8 byte unsigned integer

	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFG;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

}RCC_Reg_t;

/*******RCC peripheral Definition  **************/
#define RCC  ((RCC_Reg_t*) RCC_BASE_ADDRESS)


/*******GPIO peripheral Definition  **************/

#define GPIOA  ((GPIO_Reg_t*) GPIOA_BASE_ADDRESS)
#define GPIOB  ((GPIO_Reg_t*) GPIOB_BASE_ADDRESS)
#define GPIOC  ((GPIO_Reg_t*) GPIOC_BASE_ADDRESS)
#define GPIOD  ((GPIO_Reg_t*) GPIOD_BASE_ADDRESS)
#define GPIOE  ((GPIO_Reg_t*) GPIOE_BASE_ADDRESS)
#define GPIOF  ((GPIO_Reg_t*) GPIOF_BASE_ADDRESS)
#define GPIOG  ((GPIO_Reg_t*) GPIOG_BASE_ADDRESS)
#define GPIOH  ((GPIO_Reg_t*) GPIOH_BASE_ADDRESS)


/*******SysTick registers Definition Structure **************/

typedef struct{
	volatile uint32_t CTRL;   /*!< Offset: 0x000 (R/W) Controls the SysTick timer Register */
	volatile uint32_t LOAD;   /*!< Offset: 0x004 (R/W) Sets the reload value for the timer. Register */
	volatile uint32_t VAL;    /*!< Offset: 0x008 (R/W) Shows the current timer value Register */
	volatile const  uint32_t CALIB;  /*!< Offset: 0x00C (R/ ) Provides calibration information for the SysTick time Register */
}SysTick_Type;

/*******RCC peripheral Definition  **************/
#define SysTick  ((SysTick_Type*) SysTick_BASE_ADDRESS)



#endif /* STM32F446XX_H_ */
