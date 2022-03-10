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
#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency     */

#define PLL_CLOCK           192000000

#define F_CALC				12000			// [Hz] Control loop frequency. Must be an integer multiple of
											//4 * NETWORK_FREQ so that DELAY_ARRAY_SIZE is an integer.

#define T_CALC				(1.0/F_CALC)	// [s] Control loop trigger period. Inverse of F_CALC.

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
extern volatile uint64_t g_SysTickIntCnt;


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/
void delay_ms(uint32_t time); /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/



#endif /* MAIN_H_ */
