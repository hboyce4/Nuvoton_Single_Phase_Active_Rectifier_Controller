/*
 * main.h
 *
 *  Created on: Jan 10, 2021
 *      Author: Hugo Boyce
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_DAC //uncomment for PWM DAC

// XTAL:
// !!! Don't forget to update the __HXT macro with the correct External Crystal Clock Frequency !!!
//#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency     */

#define PLL_CLOCK           192000000



/* Interrupt priorities */
#define ADC_INT_PRIORITY 0	/* Needed for the control loop, so it has to have the highest priority*/
#define TMR1_INT_PRIORITY 1 /* Triggers the control loop, so it has to have high priority*/
#define SYSTICK_INT_PRIORITY 2
#define PDMA_INT_PRIORITY 5

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables to be made available externally                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/
//void delay_ms(uint32_t time); /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/

//void PDMA_IRQHandler(void);
//void UART_PDMATest(void);


#endif /* MAIN_H_ */
