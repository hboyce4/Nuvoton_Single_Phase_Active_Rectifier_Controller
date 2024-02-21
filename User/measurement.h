/*
 * measurement.h
 *
 *  Created on: Jan. 13, 2024
 *      Author: Hugo Boyce
 */

#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_


/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/

typedef struct measurement_inputs{ /* Process values or state variables */


	volatile float V_DC_plus;
	volatile float V_DC_minus;
	volatile float V_DC_total;
	volatile float V_DC_diff;
	volatile float v_AC;
	volatile float v_AC_n; /* v_AC normalized to a peak amplitude of one */
	volatile float i_PV;

	volatile float d; /* Current duty cycle out of the analog modulator*/
	volatile bool d_sat; /* Is the duty cycle saturated ? (low or high) */

	volatile float T_transformer;
	volatile float T_inverter;

} measurement_inputs_t;


typedef struct measurement_averages{ /* measured long-term averages at the inputs, for zeroing */

	volatile uint32_t v_Mid;
	volatile uint32_t v_AC;
	volatile uint32_t i_PV;

} measurement_averages_t;

typedef struct measurement_offsets{ /* Offsets counts for inputs and outputs*/

	// Inputs, 12 bits
	volatile uint32_t v_AC;
	volatile uint32_t i_PV;

	//Outputs, 11 bits
	volatile uint32_t i_SP;
	volatile uint32_t d_FF;

} measurement_offsets_t;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Measurement_process_oversampling(void);

void Measurement_convert_to_float(void);
void Measurement_convert_to_int_and_write(void);

void Measurement_calc_averages(void);

#endif /* MEASUREMENT_H_ */
