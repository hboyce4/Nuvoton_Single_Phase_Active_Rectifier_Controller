/*
 * timers.h
 *
 *  Created on: Jan. 6, 2024
 *      Author: Hugo Boyce
 */

#ifndef TIMERS_H_
#define TIMERS_H_



#include <stdint.h>
// Timers used:
// BPWM0 for analog PWM duty cycle capture
// BPWM1 for PWM carrier generation. Generates a square wave for the analog PWModulator
// Timer1 for generating the interrupt for the high frequency control loop
// EPWM1 for PWM-based DACs. These DACs generate signals for the duty cycle feedforward and the current setpoint

// All the timer initializations are in init.c

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

// Inverter switching frequency
#define PWM_CARRIER_FREQ 400000 //Used for BPWM1
#define BPWM0_MARGIN 20 // Number of extra clock cycles of BPWM0 on top of BPWM1 (used for BPWM0)

#define F_CALC 12000			// [Hz] Control loop frequency (used tof Timer1). Must be an integer multiple of
											//4 * NETWORK_FREQ so that DELAY_ARRAY_SIZE is an integer.
/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

extern volatile uint32_t PWM_acc_count;
extern volatile uint32_t PWM_raw_count;

#endif /* TIMERS_H_ */
