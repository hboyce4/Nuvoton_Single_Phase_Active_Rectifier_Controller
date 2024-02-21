/*
 * autozero.h
 *
 *  Created on: Sep. 26, 2023
 *      Author: Hugo Boyce
 */

#ifndef AUTOZERO_H_
#define AUTOZERO_H_

#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "NuMicro.h"
#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

// Autozero conditions
#define VBUS_MIN_FOR_AUTOZERO 20 /* [V] Minimum voltage on the busses to perform autozero */
#define COUNT_MARGIN_FOR_AUTOZERO 1750 /* [counts] Margin, in ADC counts, to consider the signal saturated (to 3.3V or GND))*/
#define FLOAT_MARGIN_FOR_AUTOZERO 0.425 /* [D] Margin, in proportion of D (1) to consider the signal saturated (to 3.3V or GND))*/

#define STARTING_GUESS_FOR_D_AUTOZERO 500 /* Starting value for doing the sweep to zero the d_FF value. The zero should be around DAC_RES_COUNT/2 */
#define LAST_GUESS_FOR_D_AUTOZERO 1500 /* Finishing value for doing the sweep to zero the d_FF value. The zero should be around DAC_RES_COUNT/2 */


/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

typedef enum {AUTOZERO_STANDBY, AUTOZERO_WAIT_FOR_CONDITIONS, AUTOZERO_I_IN_PROGRESS, AUTOZERO_D_IN_PROGRESS, AUTOZERO_DONE} autozero_state_t;

typedef struct { /* */

	volatile uint16_t guess;
	volatile int8_t bit_position;

} autozero_i_t;

typedef struct { /* */

	volatile uint16_t best_guess;
	volatile int16_t error_of_best_guess;

} autozero_d_t;

/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

extern volatile autozero_state_t autozero_state; //UI needs to be able to read this

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void autozero_state_machine(void);
bool autozero_check_conditions_ok(void);

void autozero_I_in_progress_ENTRY(void);
void autozero_I_in_progress_EXIT(void);

void autozero_D_in_progress_ENTRY(void);
void autozero_D_in_progress_EXIT(void);

#endif /* AUTOZERO_H_ */
