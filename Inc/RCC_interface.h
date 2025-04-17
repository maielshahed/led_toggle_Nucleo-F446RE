/*
 * interface.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Mai El-Shahed
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_
#include <stdint.h>



#define SET_BIT(REG, BIT) ((REG) |= (BIT))

#define  CLR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define  TOG_BIT(reg,bit)   reg^=(1<<bit)
#define  READ_BIT(REG, BIT) ((REG) & (BIT))
#define  IS_BIT_SET(reg,bit)   (reg&(1<<bit))>>bit
#define  IS_BIT_CLR(reg,bit)  !((reg&(1<<bit))>>bit)
#define  ROR(reg,num)  reg=(reg<<(READ_BIT-num))|(reg>>(num))
#define  ROL(reg,num)  reg=(reg>>(READ_BIT-num))|(reg<<(num))


//selected as system clock

enum{HSI,HSE,PLL};
/************************************************************************/

/*************************Default HSI clock value***********************/
#define HSE_VALUE 8000000U  //26MHZ --> 8 MHz, commonly used value for external crystals
extern uint32_t SystemCoreClock; // Declare it here, but not define.



#define HSI_VALUE 16000000U  /* قيمة HSI الافتراضية */
#define HSE_VALUE 8000000U   /* قيمة HSE الفعلية لديك */
#define RCC_PLLCFGR_PLLSRC_Msk (1 << PLLSRC)
#define RCC_PLLCFGR_PLLSRC_HSE (1 << PLLSRC)
#define RCC_CFGR_HPRE_Pos        4U              // Bit position of HPRE[3:0]
#


/************************************************************************/

/***************************ERROR STATUS*********************************/

#define TIMEOUT 2000UL

enum{
	RCC_ERRORSTATUS_OK,
	RCC_ERRORSTATUS_NOT_OK,
	RCC_ERRORSTATUS_TIME_OUT
};
/************************************************************************/

/**********************CLOCK STATUS**************************************/
enum{
	RCC_OFF,
	RCC_ON
};
/************************************************************************/
//selected as system clock  SW
#define RCC_CFGR_SW_HSI ((uint32_t)0x00000000)
#define RCC_CFGR_SW_HSE ((uint32_t)0x00000001)
#define RCC_CFGR_SW_PLL ((uint32_t)0x00000010)
/************************************************************************/


//CR Clock control Register
enum{HSION,HSIRDY,HSITRIM=3,HSICAL=8,HSEON =16,HSERDY,HSEBYP ,
	CSSON,PLLON=24,PLLRDY ,PLLI2SON,PLLI2SRDY ,PLLSAION,PLLSAIRDY};




/************************************************************************/

//Clock configuration register (RCC_CFGR)
enum{SW,SWS=2,HPRE=4,PPRE1=10,PPRE2=13,RTCPRE=16,
	MCO1=21 ,MCO1PRE=26 ,MCO2PRE =29,MCO2=30};




/************************************************************************/



/******************selected  System clock switch**************OR*********************/
#define SW_Pos  0         // Bit position of SW in RCC_CFGR
#define RCC_CFGR_SW_Msk (0x3 << SW_Pos) // SW[1:0] bits (System Clock Switch)

/*0x3 is the mask for the SWS field.
 * Bits 3:2 SWS[1:0]: System clock switch status

The shift (<< 2) aligns the bits correctly within the register.*/
#define RCC_CFGR_SWS_Pos       2u          // Position of SWS bits
#define RCC_CFGR_SWS_Msk       (0x3 << RCC_CFGR_SWS_Pos)   /*or  0xC====0b1100*/

/*You can use the macro along with predefined values to determine the system clock source.
 *  For better readability,
 *
#define RCC_CFGR_SWS_HSI       (0b00 <<  RCC_CFGR_SWS_Pos )
#define RCC_CFGR_SWS_HSE       (0b01 <<  RCC_CFGR_SWS_Pos )
#define RCC_CFGR_SWS_PLL       (0b11 <<  RCC_CFGR_SWS_Pos )


      or code   */
#define RCC_CFGR_SWS_HSI   (0x0)   // HSI oscillator used as system clock
#define RCC_CFGR_SWS_HSE   (0x4)   // HSE oscillator used as system clock
#define RCC_CFGR_SWS_PLL   (0x8)   // PLL used as system clock


#define SW_HSI  0b00  // HSI selected as system clock
#define SW_HSE  0b01  // HSE selected as system clock
#define SW_PLL_P 0b10  // PLL selected as system clock
#define SW_PLL_R 0b11  // selected as system clock

//selected  System clock switc
void RCC_SETSYSCLK(uint8_t SYSCLK);

/************************************************************************/





/* Bits 12:10 PPRE1: APB Low speed prescaler (APB1)***********************/
#define RCC_CFGR_APB1_Pos  10               // Starting bit position of ADC field
#define RCC_CFGR_APB1_MSK      (0x7 << 10)      // Mask for bits [15:13]  Bits 12:10 for APB2 prescaler

////Divide by 2,4,8,16
void Configure_APB1_Prescaler(uint32_t prescaler) ;
//0x7 in binary is 111, representing 3 bits.
//This defines the starting position of the PPRE1 field (bit 10).

/************************************************************************/


/* Bits 15:13 PPRE2: APB high-speed prescaler (APB2 ***********************/
#define RCC_CFGR_APB2_Pos  13               // Starting bit position of ADC field
#define RCC_CFGR_APB2_MSK      (0x7 << 13)      // Mask for bits [15:13]  Bits 15:13 for APB2 prescaler
#define div_AHBclk2 0b100
#define div_AHBclk4 0b101
#define div_AHBclk8 0b110
#define div_AHBclk16 0b111
void APB2_prescaler(uint32_t prescaler);
//0x7 in binary is 111, representing 3 bits.
//This defines the starting position of the PPRE1 field (bit 13).


/************************************************************************/



/*Bits 7:4 HPRE: AHB prescaler  ************************************/
#define RCC_CFGR_AHB_Pos      4u               // Starting bit position of ADC field
#define RCC_CFGR_AHB_MSK      (0xF << 4)      // Mask for bits [7:4]

#define div_sysclk2          0b1000     /*    0x08    Divide by 2 */
#define div_sysclk4          0b1001     /*    0x09    Divide by 4 */
#define div_sysclk8          0b1010    /*    0x10    Divide by 8 */
#define div_sysclk16         0b1011    /*    0x11    Divide by 16 */
#define div_sysclk64         0b1100
#define div_sysclk128        0b1101
#define div_sysclk256        0b1110
#define div_sysclk2512       0b1111

void AHB_prescaler(uint32_t prescaler);
//0xF in binary is 1111, representing 4 bits.
//This defines the starting position of the AHB field (bit 4).


/************************************************************************/


//6.3.10 RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
enum{GPIOAEN,GPIOBEN,GPIOCEN,GPIODEN,GPIOEEN,GPIOFEN,GPIOGEN,GPIOHEN,CRCEN=12,BKPSRAMEN=18,
	DMA1EN=21,DMA2EN,OTGHSEN=29,OTGHSULPIEN};

void RCC_AHB1_enable(uint32_t peripheral);
void RCC_AHB1_disable(uint32_t peripheral);


/************************************************************************/
//6.3.11 RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
enum{DCMIEN,OTGFSEN=7};

void RCC_AHB2_enable(uint32_t peripheral);
void RCC_AHB2_disable(uint32_t peripheral);

/************************************************************************/

//6.3.12 RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
enum{FMCEN,QSPIEN};
void RCC_AHB3_enable(uint32_t peripheral);
void RCC_AHB3_disable(uint32_t peripheral);

/************************************************************************/
//6.3.13 RCC APB1 peripheral clock enable register (RCC_APB1ENR)
enum{TIM2EN,TIM3EN,TIM4EN,TIM5EN,TIM6EN,TIM7EN,TIM12EN,TIM13EN,TIM14EN,WWDGEN=11,
	SPI2EN=14,SPI3EN,SPDIFRXEN,USART2EN,USART3EN,UART4EN,UART5EN,I2C1EN,I2C2EN,I2C3EN,
	FMPI2C1EN,CAN1EN,CAN2EN,CECEN,PWREN,DACEN};
void RCC_APB1_enable(uint32_t peripheral);
void RCC_APB1_disable(uint32_t peripheral);

/************************************************************************/
//6.3.14 RCC APB2 peripheral clock enable register (RCC_APB2ENR)

enum{TIM1EN,TIM8EN,USART1EN=4,USART6EN,ADC1EN=8,ADC2EN,ADC3EN,SDIOEN,SPI1EN,SPI4EN,
	SYSCFGEN,TIM9EN = 16,TIM10EN,TIM11EN,SAI1EN=22,SAI2EN};

void RCC_APB2_enable(uint32_t peripheral);
void RCC_APB2_disable(uint32_t peripheral);

/************************************************************************/






//selected enable as system clock  OR TIMEOUT
uint8_t RCC_SETCLK(uint8_t CLKtype, uint8_t CLKstatus);

void RCC_enableClk(uint8_t clktype);
void RCC_disableClk(uint8_t clktype);


/******************RCC PLL configuration register (RCC_PLLCFGR*******************************************/
enum{PLLM,PLLN=6,PLLP=16,PLLSRC=22,PLLQ=24,PLLR=28};

#define RCC_PLLCFGR_PLLM_Pos 0U     // PLLM bit position

#define RCC_PLLCFGR_PLLM_Msk (0x3F << RCC_PLLCFGR_PLLM_Pos)  // Mask for PLLM (6 bits)




/*
 *   Division   /8U
 *
 * RCC->PLLCFGR |= (8U << RCC_PLLCFGR_PLLM_Pos);  // Set PLLM = 8
 *
 * */
/************************************************************************/

////PLLN[8:0]: Main PLL (PLL) multiplication factor for
#define RCC_PLLCFGR_PLLN_Pos 6U    // LSB position of PLL multiplier
/*0x1FF (binary: 111111111) represents 9 bits.
The mask isolates bits [14:6] by shifting 0x1FF to the left by 6 (value of RCC_PLLCFGR_PLLN_Pos).*/
#define RCC_PLLCFGR_PLLN_Msk (0x1FF << RCC_PLLCFGR_PLLN_Pos)     /*Mask for PLLN (9 bits)*/


/*
 * multiplication    *  80
 * RCC->PLLCFGR |= (80U << RCC_PLLCFGR_PLLN_Pos);  // Set PLLN = 80
 *
 * */
void RCC_PLLconfig(  uint8_t clksrc);
void ConfigurePLL(uint32_t pll_n) ;
/************************************************************************/

/**************PLLM[5:0]: Division factor for the main PLL**************************/
//PLL configuration PLLM[5:0]: Division factor for the main PLL

void Configure_PLLM(uint8_t division_factor);

/************************************************************************/


//PLLP[1:0]: Main PLL (PLL) division 2-/-8factor for main system clock

#define PLLP_Pos  16               // Starting bit position of PLLP field
#define PLLCFGR_PLLP      (0x3 << 16)      // Mask for bits [ 17:16]
#define div_PLLPclk2 0b00
#define div_PLLPclk4 0b01
#define div_PLLPclk6 0b10
#define div_PLLPclk16 0b11

void Configure_PLLP(uint8_t division_factor);
void Configure_orPLLP(uint8_t ORdivision_factor);
/*0x3 in binary is 11, representing 2 bits.
This defines the starting position of the PLLP field (bit 16).

    division /2
    RCC->PLLCFGR |= (0U << RCC_PLLCFGR_PLLP_Pos);  // Set PLLP = 2



************************************************************************/
void RCC_EnablePLL(void);

void SystemCoreClockUpdate(void);
#endif /* INTERFACE_H_ */
