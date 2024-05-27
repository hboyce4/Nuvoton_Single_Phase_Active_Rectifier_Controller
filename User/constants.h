/*
 * constants.h
 *
 *  Created on: Mar 10, 2022
 *      Author: Hugo Boyce
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "timers.h"
/*---------------------------------------------------------------------------------------------------------*/
/* Global Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#define T_CALC ((float)(1.0/F_CALC))	// [s] Control loop trigger period. Inverse of F_CALC.

#define T_CACA 5

#define T_SYSTICK 0.001f //[s] 1 millisecond systick


#define ADC_RES_BITS 12
#define ADC_RES_COUNT 4096.0f // 2^ADC_RES_BITS

#define DAC_RES_BITS 11
#define DAC_RES_COUNT 2048.0f // 2^DAC_RES_BITS



// Nominal voltages
#define V_AC_NOMINAL_RMS_VALUE 12.0f 	/* Nominal RMS value of the input voltage (ex.: 12V AC rms)*/
#define VBUS_DIFF_DEFAULT 0.0f /* [V] Default voltage difference setpoint between the two half-busses*/
#define VBUS_TOTAL_DEFAULT 48.0f /* [V] Default total bus voltage setpoint*/

#endif /* CONSTANTS_H_ */
