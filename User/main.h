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


#define PLL_CLOCK           192000000
#define F_CALC				12000						// [HZ] Fréquence de traitement du signal. doit être un multiple entier de 4 * NETWORK_FREQ pour que DELAY_ARRAY_SIZE soit un entier.
#define T_CALC				(1.0/F_CALC)				// [s] Période de calcul du traitement de signal. Inverse de F_CALC

/* Interrupt prioritires */

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




#endif /* MAIN_H_ */
