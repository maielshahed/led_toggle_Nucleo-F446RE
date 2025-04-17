/*
 * GPIO_interface.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Mai El Shahed
 */

#ifndef GPIO_INTERFACE_H_
#define GPIO_INTERFACE_H_
#include "errtype.h"
/***********************************************
 * @port_t enum
 *
 *
 */
typedef enum{
	PORTA=0,
	PORTB,
	PORTC,
	PORTD,
	PORTE,
	PORTF,
	PORTG,
	PORTH
}port_t;
/***************************************************
 * @pin_t enum
 *
 *
 */
typedef enum{
	PIN0=0,
	PIN1,
	PIN2,
	PIN3,

	PIN4,
	PIN5,
	PIN6,
	PIN7,

	PIN8,
	PIN9,
	PIN10,
	PIN11,

	PIN12,
	PIN13,
	PIN14,
	PIN15

}pin_t;

typedef enum{
	INPUT=0,
	OUTPUT,
	ALTERNATE_FUNCTION,
	ANALOG
}mode_t;


typedef enum{
	LOW=0,    //00   BIT 0
	MEDIUM,    //01  BIT 1
	FAST,     //10   BIT 2
	HIGH      //11     BIT 3

}outputspeed_t;

typedef enum{
	PUSH_PULL=0,
	OPEN_DEAIN
}outputtype_t;

typedef enum{
	NOPULL=0,
	PULLUP,
	PULLDOWN

}pullupdown_t;

/***************************************************
 * @pinval_t enum
 *
 *
 */
typedef enum{
	PIN_LOW=0,
	PIN_HIGH
}pinval_t;

typedef enum{
	AF0=0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15

}altfunc_t;


typedef struct{
	port_t port;
	pin_t pinNum;
 	mode_t mode;
	outputspeed_t speed;
	outputtype_t outputtype;
	pullupdown_t pulltype;
	altfunc_t altfunc;

}pin_configer_t;

uint8_t GPIO_u8pininit(const pin_configer_t* pin_config);

uint8_t GPIO_u8setpinvalue(port_t port,pin_t pinNum,pinval_t pinval);

uint8_t GPIO_u8togglepinvalue(port_t port,pin_t pinNum);

uint8_t GPIO_u8readpinvalue(port_t port,pin_t pinNum,pinval_t* pinval);



#endif /* GPIO_INTERFACE_H_ */
