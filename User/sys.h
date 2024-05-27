/*
 * sys.h
 *
 *  Created on: Apr 4, 2022
 *      Author: Hugo Boyce
 */

#ifndef SYS_H_
#define SYS_H_

#include <stdint.h>
#include "interrupt.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#define GREEN_LED_PIN PA12
#define AMBER_LED_PIN PA13
#define RED_LED_PIN PA14
#define BLUE_LED_PIN PA15

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/
void delay_ms(uint32_t time); /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/

#endif /* SYS_H_ */
