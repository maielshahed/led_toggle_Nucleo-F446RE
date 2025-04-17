Project Description (STM32 Nucleo-F446RE)

This project initializes the STM32 Nucleo-F446RE microcontroller using the internal HSI clock and toggles the onboard LED (connected to pin PA5) every second.

Key Features: 
Uses HSI (High-Speed Internal) clock as the system clock source. 
Configures SysTick for delay functions. Enables the GPIOA peripheral clock. 
Configures PA5 as a push-pull output pin (User LED on the Nucleo board). 
Toggles the LED with a delay of 1 second ON and 1 second OFF.

Peripherals Used 
RCC: System clock configuration 
SysTick: Software delays 
GPIOA Pin 5: Output for onboard LED
