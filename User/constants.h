/*
 * constants.h
 *
 *  Created on: Mar 10, 2022
 *      Author: Hugo Boyce
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Global Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

// Inverter switching frequency
#define PWM_CARRIER_FREQ 400000

#define F_CALC 12000			// [Hz] Control loop frequency. Must be an integer multiple of
											//4 * NETWORK_FREQ so that DELAY_ARRAY_SIZE is an integer.

#define T_CALC (1.0/F_CALC)	// [s] Control loop trigger period. Inverse of F_CALC.


#define T_SYSTICK (1.0/1000) // [s] 1 millisecond systick


#define RES_12BIT 4096.0
#define RES_11BIT 2048.0


// Nominal voltages
#define V_AC_NOMINAL_RMS_VALUE 12 	/* Nominal RMS value of the input voltage (ex.: 12V AC rms)*/
#define VBUS_DIFF_DEFAULT 0 /* [V] Default voltage difference setpoint between the two half-busses*/
#define VBUS_TOTAL_DEFAULT 48 /* [V] Default total bus voltage setpoint*/

#endif /* CONSTANTS_H_ */
