/*
 * autozero.c
 *
 *  Created on: Sep. 26, 2023
 *      Author: Hugo Boyce
 */

#include "autozero.h"
#include <stdint.h>
#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile autozero_state_t autozero_state;
volatile autozero_i_t autozero_i;
volatile autozero_d_t autozero_d;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/





void autozero_state_machine(void){

//TODO: Add digital duty cycle feedback to be able to zero d_FF output as well. Analog duty cycle feedback would become also obsolete.

/**************State machine for autozero conditions**************************************************/



	/* For the autozero to take place, some other function must bring the voltage high enough,
	 *  open the relays, and set the state to AUTOZERO_WAIT_FOR_CONDITIONS */
	bool conditions_ok = autozero_check_conditions_ok();

	if((!conditions_ok) && autozero_state != AUTOZERO_DONE){ /* If conditions not ok and the autozero isn't finished*/
		autozero_state = AUTOZERO_STANDBY; // Reset
		// AUTOZERO_STANDBY state entry code here
	}

	switch(autozero_state){

		case AUTOZERO_STANDBY:
			// Do nothing
			break;

		case AUTOZERO_WAIT_FOR_CONDITIONS:

			if(conditions_ok){

				// AUTOZERO_WAIT_FOR_CONDITIONS state exit code would go here

				autozero_I_in_progress_ENTRY();


			}
			//
			break;

		case AUTOZERO_I_IN_PROGRESS:

			if(autozero_i.bit_position >= 0){ /* If there are bit positions left to evaluate, that is, bit position is not negative*/

				if(COMP_RESET_PIN){/* If compensator reset is active*/
					COMP_RESET_PIN = false; /* remove the compensator reset (it was active for one cycle), and wait for the next execution, to give time to the compensator to saturate*/
				}else{
					if(measurements_in.d > (1.0-FLOAT_MARGIN_FOR_AUTOZERO)/*ADC_raw_val[D_COMP_CHANNEL] > ((uint16_t)(ADC_RES_COUNT - COUNT_MARGIN_FOR_AUTOZERO))*/){/* If d_COMP saturated high*/
						/* That means the guess was too high */
						/* Put the bit back to zero in the guess*/
						autozero_i.guess &= ~(1 << autozero_i.bit_position);
						autozero_i.bit_position--;/* Go to next bit */

						if(autozero_i.bit_position >= 0){/* If there are bit positions left to evaluate, that is, bit position is not negative*/
							/* Prepare the next guess*/
							autozero_i.guess |= (1 << autozero_i.bit_position);/* set bit to 1 in guess at new bit position*/
							DAC_write_i_SP((uint32_t)autozero_i.guess);/* Write the guess to the DAC*/
							COMP_RESET_PIN = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/
						}


					}else if(measurements_in.d < FLOAT_MARGIN_FOR_AUTOZERO/*ADC_raw_val[D_COMP_CHANNEL] < ((uint16_t)COUNT_MARGIN_FOR_AUTOZERO)*/){/* If d_COMP saturated low*/
						/* That means the guess was too low */
						/* Keep the bit at 1 in the guess*/
						autozero_i.bit_position--;/* Go to next bit */

						if(autozero_i.bit_position >= 0){/* If there are bit positions left to evaluate, that is, bit position is not negative*/
							/* Prepare the next guess*/
							autozero_i.guess |= (1 << autozero_i.bit_position); /* set bit to 1 in guess at new bit position*/
							DAC_write_i_SP((uint32_t)autozero_i.guess); /* Write the guess to the DAC*/
							COMP_RESET_PIN = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/
						}



					}else{/**/
						// Else do nothing and wait for next cycle, leave some time for the compensator to saturate
					}
				}

			}else{/* Else there aren't any bit positions left to try, the autozero is finished*/

				// AUTOZERO_IN_PROGRESS state exit code here
				autozero_I_in_progress_EXIT();


				// AUTOZERO_D_IN_PROGRESS state entry code here
				autozero_D_in_progress_ENTRY();


			}

			break;


		case AUTOZERO_D_IN_PROGRESS:
			// Zeroing of the duty cycle at 50% during compensator reset using the d_FF offset value
			uint32_t error;

			error = abs(PWM_raw_count-(BPWM_GET_CNR(BPWM1,0)/2));

			if(error < autozero_d.error_of_best_guess){
				autozero_d.error_of_best_guess = error;
				autozero_d.best_guess = DAC_read_d_FF();
			}

			DAC_write_d_FF(DAC_read_d_FF()+1); /* Increment d_FF by one*/


			if(/*Exit condition: end of sweep reached*/){
				// AUTOZERO_D_IN_PROGRESS state exit code here
				autozero_D_in_progress_EXIT();

				// AUTOZERO_DONE state entry code here
				autozero_state = AUTOZERO_DONE;
			}


			break;


		case AUTOZERO_DONE:
			//
			break;

		default: // If un-handled state, that's an error
			autozero_state = AUTOZERO_STANDBY; //Reset
	}

}


bool autozero_check_conditions_ok(void){
	/* Voltages on the busses high enough that no current is drawn on the xformer and the AC relay and precharge are open (high) */
	return (measurements_in.V_DC_plus > VBUS_MIN_FOR_AUTOZERO) && (measurements_in.V_DC_minus < -VBUS_MIN_FOR_AUTOZERO) && PC2 && PC4;
}


void autozero_I_in_progress_ENTRY(void){

	autozero_state = AUTOZERO_I_IN_PROGRESS;// Go to next state
	// AUTOZERO_IN_PROGRESS state entry code
	autozero_i.bit_position = DAC_RES_BITS - 1;
	autozero_i.guess = ((uint16_t)DAC_RES_COUNT)>>1; /* Initial guess, half the full DAC range */
	DAC_write_i_SP((uint32_t)autozero_i.guess);
	COMP_RESET_PIN = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/

}

void autozero_I_in_progress_EXIT(void){

	measurement_offsets.i_SP = autozero_i.guess; // Save the value found
	measurement_offsets.v_AC = measurement_avgs.v_AC; /* Zero the average voltage*/
	measurement_offsets.i_PV = measurement_avgs.i_PV; /* Zero the input current*/

}

void autozero_D_in_progress_ENTRY(void){
	autozero_state = AUTOZERO_D_IN_PROGRESS;

	DAC_write_d_FF((uint32_t)STARTING_GUESS_FOR_D_AUTOZERO); /* Set the d_FF output to the starting guess*/
	autozero_d.error_of_best_guess = INT16_MAX; /*Largest possible value to start */

	COMP_RESET_PIN = true; /* Put the compensator in the reset state*/
	/* Set the COMP reset pin!!! */

}

void autozero_D_in_progress_EXIT(void){


}
