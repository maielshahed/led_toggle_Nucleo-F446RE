/*
 * RCC_Driver.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Mai El-Shahed
 */

#include "RCC_interface.h"
#include "Stm32F446xx.h"
#include "errtype.h"
#include <stdint.h>


uint32_t SystemCoreClock = 16000000; // Or your system core clock value.


/*****************RCC SET CLocK*************selected enable as system clock  OR TIMEOUT*****************************/
uint8_t RCC_SETCLK(uint8_t CLKtype, uint8_t CLKstatus)
{
    uint8_t RCC_ERRORSTATUS = RCC_ERRORSTATUS_OK; // Default error status==0
    uint32_t TIME = TIMEOUT;                     // Timeout counter
    switch (CLKtype)
    {
		case HSI:
		{
			if (CLKstatus == RCC_ON)
			{
				//SET_BIT ((RCC->CR), HSION);
				RCC->CR |= (1 << HSION); // Enable HSI
				while (((RCC->CR & (1 << HSIRDY)) == RCC_OFF) && (--TIME)); // Wait for HSI to stabilize
			}
			else
			{
				RCC->CR &= ~(1 << HSION); // Disable HSI
	        	//CLR_BIT ((RCC->CR), HSION);

			}
			break;
		}
		case HSE:
		{
			if (CLKstatus == RCC_ON)
			{
				RCC->CR |= (1 << HSEON); // Enable HSE
				while (((RCC->CR & (1 << HSERDY)) == RCC_OFF) && (--TIME)); // Wait for HSI to stabilize
			}
			else
			{
				RCC->CR &= ~(1 << HSEON); // Disable HSE
			}
			break;
		}
		case PLL:
		{
			if (CLKstatus == RCC_ON)
			{

				RCC->CR |= (1 << PLLON); // Enable PLL
				while (((RCC->CR & (1 << PLLRDY)) == RCC_OFF) && (--TIME)); // Wait for HSI to stabilize
			}
			else
			{
				RCC->CR &= ~(1 << PLLON); // Disable PLL
			}
			break;
		}
		default:
			RCC_ERRORSTATUS = RCC_ERRORSTATUS_NOT_OK; // Invalid clock type
    }
    // Check if timeout occurred
    if (TIME == 0)
    {
        RCC_ERRORSTATUS = RCC_ERRORSTATUS_TIME_OUT;
    }

    return RCC_ERRORSTATUS;
}
/************************************************************************/

//OR selected as system clock
void RCC_enableClk(uint8_t clktype) {
    switch (clktype) {

        case HSI:
 // Use HSI (High-Speed Internal) as the system clock
 // Enable the desired clock source and wait for it to stabilize

        	 // Enable HSI; // Enable HSI
        	SET_BIT ((RCC->CR), HSION);
        	while(!(READ_BIT((RCC->CR),HSIRDY)));// Wait for HSI to stabilize
            break;

        case HSE:
            // Use HSE (High-Speed External) as the system clock
        	SET_BIT ((RCC->CR), HSEON); // Enable HSE

        	while(!(READ_BIT((RCC->CR),HSERDY))); // Wait for HSI to stabilize

            break;
        case PLL:


            SET_BIT ((RCC->CR), PLLON); // Enable PLL
            while (!(RCC->CR & PLLRDY)); // Wait for HSI to stabilize

            break;

        default:

            // Error handling can be added here
            break;
    }



}




//OR selected disable as system clock

void RCC_disableClk(uint8_t clktype) {
    switch (clktype) {

        case HSI:
 // Use HSI (High-Speed Internal) as the system clock
 // clear the desired clock source

        	 // clear HSI; // Enable HSI
        	CLR_BIT ((RCC->CR), HSION);

            break;

        case HSE:
            // Use HSE (High-Speed External) as the system clock
        	CLR_BIT ((RCC->CR), HSEON); // clear HSE
            break;
        case PLL:


           CLR_BIT ((RCC->CR), PLLON); // clear PLL

            break;

        default:

            // Error handling can be added here
            break;
    }

}






//PLL configuration OR HSI- HSE

//PLL configuration OR HSI- HSE

void RCC_PLLconfig(uint8_t clksrc){
    switch (clksrc) {
        case HSI:
            SET_BIT(RCC->CR, HSION);                 // Enable HSI
            while (!(RCC->CR & (1 << HSIRDY)));      // Wait until HSI is ready
            CLR_BIT(RCC->PLLCFGR, PLLSRC);           // Select HSI as PLL source
            break;
        case HSE:
            SET_BIT(RCC->CR, HSEON);                 // Enable HSE
            while (!(RCC->CR & (1 << HSERDY)));      // Wait until HSE is ready
            SET_BIT(RCC->PLLCFGR, PLLSRC);           // Select HSE as PLL source
            break;
        default:
            return; // Error
    }
}
/// Configure the PLL multiplication
void ConfigurePLL(uint32_t pll_n) {
    // Ensure PLLN is within the valid range (e.g., 50 <= PLLN <= 432 for STM32F4)
    if (pll_n < 50 || pll_n > 432) {
        // Handle invalid value
        return;
    }

    // Disable the PLL before configuration

    RCC->CR &= ~(1 << PLLON);  // Clear the PLLON bit to disable the PLL
      while (RCC->CR & (1 << PLLRDY));  // Wait until PLL is fully disabled



    // Configure the PLL multiplication factor (PLLN[8:0])
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_Msk);       // Clear the PLLN bits
    RCC->PLLCFGR |= (pll_n << RCC_PLLCFGR_PLLN_Pos); // Set the desired PLLN value

    // Re-enable the PLL
    RCC->CR |= (1 << PLLON);

    // Wait until the PLL is ready
    while (!(RCC->CR & (1 << PLLRDY)));  // Wait until PLL is fully ready
}

//PLL configuration PLLM[5:0]: Division factor for the main PLL


void Configure_PLLM(uint8_t division_factor) {
    // Ensure the division factor is within the valid range (1 to 63)
    if (division_factor < 1 || division_factor > 63) {
        return;  // Invalid input, exit the function
    }


    // Disable the PLL before making any changes
    RCC->CR &= ~(1 << PLLON);  // Clear the PLLON bit to disable the PLL
    while (RCC->CR & (1 << PLLRDY));  // Wait until PLL is fully disabled

    // Configure the PLLM division factor
    RCC->PLLCFGR &= ~(1 << PLLM);          // Clear the PLLM[5:0] bits
    RCC->PLLCFGR |= (division_factor << 0);     // Set the new division factor

    // Re-enable the PLL
    RCC->CR |= (1 << PLLON);  // Set the PLLON bit to enable the PLL
    while (!(RCC->CR & (1 << PLLRDY)));  // Wait until PLL is fully ready
}


// PLLP division factor must be 2, 4, 6, or 8 in STM32 series
void Configure_PLLP(uint8_t division_factor) {
    // PLLP division factor must be 2, 4, 6, or 8 in STM32 series
    if (division_factor != 2 && division_factor != 4 &&
        division_factor != 6 && division_factor != 8) {
        return;  // Invalid input, exit the function
    }

    // Disable the PLL before modifying its configuration
    RCC->CR &= ~(1 << PLLON);  // Clear the PLLON bit to disable the PLL
    while (RCC->CR & (1 << PLLRDY));  // Wait until PLL is fully disabled

    RCC->PLLCFGR &= ~(0b11 << PLLP );// Clear PLLP bits
    RCC->PLLCFGR |= ((division_factor / 2 - 1) << 16);

    // Configure the PLLP division factor


    // Re-enable the PLL
    RCC->CR |= (1 << PLLON);  // Set the PLLON bit to enable the PLL
    while (!(RCC->CR & (1 << PLLRDY)));  // Wait until PLL is fully ready
}

//OR   PLLP division factor must be div_PLLPclk2 2, 4, 6, or 8 in STM32 series

void Configure_orPLLP(uint8_t ORdivision_factor){
	 // Disable the PLL before modifying its configuration


	    RCC->CR &= ~(1 << PLLON);  // Clear the PLLON bit to disable the PLL
	    while (RCC->CR & (1 << PLLRDY)){
	    	// Wait until PLL is fully disabled
	    }


		 RCC->PLLCFGR &= ~ (PLLCFGR_PLLP);  //Clear PLLP bits Mask for bits [ 17:16]
		 RCC->PLLCFGR|=(ORdivision_factor << PLLP_Pos );////Set div_PLLPclk2 to divide

	    // Re-enable the PLL
	     RCC->CR &= ~(1 << PLLON);  // Set the PLLON bit to enable the PLL
	     while (!(RCC->CR & (1 << PLLRDY)));  // Wait until PLL is fully ready
}


/******************selected  System clock switch**************OR*********************/
volatile uint32_t timeout = 0xFFFF;

void RCC_SETSYSCLK(uint8_t SYSCLK) {
        // Clear the SW[1:0] bits first

    RCC->CFGR &= ~(RCC_CFGR_SW_Msk);

    // Set the new clock source
    RCC->CFGR |= (SYSCLK << SW_Pos);

    // Wait until the new system clock source is used
        while (((RCC->CFGR & RCC_CFGR_SWS_Msk) >>RCC_CFGR_SWS_Pos ) != SYSCLK){
            // يمكن إضافة timeout لمنع التجميد
            if (--timeout == 0) break;
        }
    }


/************************************************************************/
/*dynamically calculates SystemCoreClock based on the current RCC register configuration.
 *  This is updated by the SystemCoreClockUpdate() function.
 *  ************************/

//
void SystemCoreClockUpdate(void) {
    uint32_t tmp, pllm, pllvco, pllp;
    uint32_t sysclk_source;
/*This binary mask (0b1100) is used to isolate the SWS bits
 * when reading the RCC_CFGR registe*/
    sysclk_source = RCC->CFGR & RCC_CFGR_SWS_Msk ;

    if (sysclk_source == RCC_CFGR_SWS_HSI) { // HSI used as system clock
        SystemCoreClock = 16000000;

    } else if (sysclk_source == RCC_CFGR_SWS_HSE) { // HSE used as system clock

    	SystemCoreClock = HSE_VALUE;


    } else if (sysclk_source == RCC_CFGR_SWS_PLL) { // PLL used as system clock


    	/*OR  pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk)      >>  RCC_PLLCFGR_PLLM_Pos   );*/
    	pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk;

        /*Getting the Actual PLLN Value: To get the value as an integer,
         *  you need to shift it to the right:
         * */
        pllvco = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos) * (HSE_VALUE / pllm);

        /*If the bits are 01 (encoded as 1), the calculation becomes (1 + 1) * 2 = 4   Result: pllp = 4.*/
        pllp = (((RCC->PLLCFGR & PLLCFGR_PLLP) >> PLLP_Pos) + 1) * 2;

        SystemCoreClock = pllvco / pllp;
    }




    /*Bits 7:4 HPRE: AHB prescaler  ************************************/
    tmp = ((RCC->CFGR & RCC_CFGR_AHB_MSK) >> RCC_CFGR_AHB_Pos);
/*If tmp is 8 or greater, the AHB clock is divided.
The division factor is determined by (tmp - 0x07). */
    if (    tmp >= 0x08    ) {

        SystemCoreClock >>= (tmp - 0x07);
    }
}


void RCC_EnablePLL(void) {


    RCC->CR |= (1 << PLLON);  // Set the PLLON bit to enable the PLL
       while (!(RCC->CR & (1 << PLLRDY)));  // Wait until PLL is fully ready
}




/************************************************************************/
/****************** AHB1 peripheral clock enable&disabled*********************************/


void RCC_AHB1_enable(uint32_t peripheral){
	RCC->AHB1ENR |=(1 << peripheral);
}
void RCC_AHB1_disable(uint32_t peripheral){
	RCC->AHB1ENR &=~(1 << peripheral);
}
/************************************************************************/

/****************AHB2 peripheral clock enable&disabled**************************/
void RCC_AHB2_enable(uint32_t peripheral){
	RCC->AHB2ENR |=(1 << peripheral);
}
void RCC_AHB2_disable(uint32_t peripheral){
	RCC->AHB2ENR &=~(1 << peripheral);
}
/************************************************************************/

/****************AHB3 peripheral clock enable&disabled**************************/
void RCC_AHB3_enable(uint32_t peripheral){
	RCC->AHB3ENR |=(1 << peripheral);
}
void RCC_AHB3_disable(uint32_t peripheral){
	RCC->AHB3ENR &=~(1 << peripheral);
}
/************************************************************************/


/****************APB1 peripheral clock enable&disabled**************************/
void RCC_APB1_enable(uint32_t peripheral){
	RCC->APB1ENR |=(1 << peripheral);
}
void RCC_APB1_disable(uint32_t peripheral){
	RCC->APB1ENR &=~(1 << peripheral);
}
/************************************************************************/

/****************APB2 peripheral clock enable&disabled**************************/
void RCC_APB2_enable(uint32_t peripheral){
	RCC->APB2ENR |=(1 << peripheral);
}
void RCC_APB2_disable(uint32_t peripheral){
	RCC->APB2ENR &=~(1 << peripheral);
}
/************************************************************************/


// APB2 high-speed prescaler


void APB2_prescaler(uint32_t prescaler){
	RCC->CFGR &=~(RCC_CFGR_APB2_MSK);// Clear current prescaler Mask for bits [15:13]

	RCC->CFGR |=(prescaler << RCC_CFGR_APB2_Pos);//Set prescaler to divide by prescaler

}

/************************************************************************/


// APB1 high-speed prescaler
//Divide by 2,4,8,16

void Configure_APB1_Prescaler(uint32_t prescaler) {
    uint32_t prescaler_bits = 0;

    // Map prescaler value to PPRE2 bits
    switch (prescaler) {
        case 0:  prescaler_bits = 0x0; break;  // No division
        case 2:  prescaler_bits = 0x4; break;  // Divide by 2 ----->0b100
        case 4:  prescaler_bits = 0x5; break;  // Divide by 4  ----->0b101
        case 8:  prescaler_bits = 0x6; break;  // Divide by 8  ----->0b110
        case 16: prescaler_bits = 0x7; break;  // Divide by 16   ----->0b111
        default: return;  // Invalid prescaler
    }

    // Configure RCC_CFGR for PPRE2
    RCC->CFGR &= ~(RCC_CFGR_APB1_MSK);            // Clear bits 12:10
    RCC->CFGR |= (prescaler_bits << RCC_CFGR_APB1_Pos);     // Set new prescaler value
}
/************************************************************************/


//AHB prescaler

void AHB_prescaler(uint32_t prescaler){


	uint32_t hpre_bits = 0;

	    // Map prescaler value to HPRE bits
	    switch (prescaler) {
	        case 0:  hpre_bits = 0x0; break;  // No division
	        case 2:  hpre_bits = 0x8; break;  // Divide by 2  ----->0b1000
	        case 4:  hpre_bits = 0x9; break;  // Divide by 4  ------>0b1001
	        case 8:  hpre_bits = 0xA; break;  // Divide by 8 -----..>0b1010
	        case 16: hpre_bits = 0xB; break;  // Divide by 16 -----..>0b1011
	        case 64: hpre_bits = 0xC; break;  // Divide by 64 -----..>0b1100
	        case 128: hpre_bits = 0xD; break; // Divide by 128   -----..>0b1101
	        case 256: hpre_bits = 0xE; break; // Divide by 256   -----..>0b1110
	        case 512: hpre_bits = 0xF; break; // Divide by 512    -----..>0b1111
	        default: return;  // Invalid prescaler
	    }
	RCC->CFGR &=~(RCC_CFGR_AHB_MSK);// Clear current prescaler Mask for bits [7:4]

	RCC->CFGR |=(hpre_bits << RCC_CFGR_AHB_Pos);//Set prescaler to divide by prescaler

}

/************************************************************************/










