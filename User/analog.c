/*
 * analog.c
 *
 *  Created on: Feb 6, 2021
 *      Author: Hugo Boyce
 */

#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile analog_inputs_t analog_in; /* directly measured values */
volatile uint32_t ADC_raw_val[EADC_TOTAL_CHANNELS];
volatile uint16_t ADC_acq_buff[EADC_TOTAL_CHANNELS];
volatile uint8_t ADC_acq_count;

volatile analog_averages_t analog_avgs;

volatile analog_offsets_t analog_offsets = {.v_AC = V_AC_OFFSET,
											.i_PV = I_PV_OFFSET,
											.i_SP = I_SP_OFFSET,
											.d_FF = D_FF_OFFSET};

volatile autozero_state_t autozero_state;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
// TODO: Add conversion to celsius  for the temperature channels

void run_ADC_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk|EADC_CALCTL_CALSEL_Msk;/* Set CALSTART to start calibration. Set CALSEL so it doesn't try to load user calibration word*/
	/* EADCDIV must be zero to run calibration */
}

void ADC_service_oversampling(void){

	/* Divide,copy and reset */
	uint8_t channel;
	for(channel = 0; channel < EADC_TOTAL_CHANNELS;channel++){/* For every channel */

		/* this step could be skipped by integrating this calculation in the float conversion step*/
		ADC_raw_val[channel]= (ADC_acq_buff[channel] >> EADC_SHIFT_FOR_OVERSAMPLING_DIVISION); /* Copy the acquisition buffer, dividing by the number of oversamples. */

		ADC_acq_buff[channel] = 0;
	}

	/* Start ADC acquisition batch*/
	ADC_acq_count = EADC_OVERSAMPLING_NUMBER;

	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	NVIC_EnableIRQ(EADC00_IRQn);

}



void ADC_convert_to_float(void){

	/* Vbus plus */
	analog_in.V_DC_plus = ((float)ADC_raw_val[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*VBUS_PLUS_GAIN))-VBUS_PLUS_OFFSET;
	/* Vbus minus */
	analog_in.V_DC_minus = -(((float)ADC_raw_val[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*VBUS_MINUS_GAIN)))-VBUS_MINUS_OFFSET;
	/* Vbus total */
	analog_in.V_DC_total = analog_in.V_DC_plus - analog_in.V_DC_minus;
	/* Vbus diff */
	analog_in.V_DC_diff = analog_in.V_DC_plus + analog_in.V_DC_minus;
	/* V AC */
	analog_in.v_AC = ((float)ADC_raw_val[V_AC_CHANNEL] - (float)analog_offsets.v_AC)*(VREF_VOLTAGE/(ADC_RES_COUNT*V_AC_GAIN));
	/* V AC normalized to 1 */
	analog_in.v_AC_n = analog_in.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*M_SQRT2)); /* needs math.h */
	/* Current process value (actual amperage) */
	analog_in.i_PV = ((float)ADC_raw_val[I_PV_CHANNEL] - (float)analog_offsets.i_PV)*(VREF_VOLTAGE/(ADC_RES_COUNT*I_PV_GAIN));
}



void convert_to_int_write_analog(void){

	int32_t i_sp_val, d_ff_val;

	/* Convert the current setpoint from float to int*/
	i_sp_val = 	(int32_t)(inverter.i_SP*I_SP_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE));
	i_sp_val += (int32_t)analog_offsets.i_SP; /* signal centered around I_SP_OFFSET */

	/* Convert the duty cycle feedforward value from int to float */
	d_ff_val = (int32_t)((inverter.d_feedforward-0.5)*D_FF_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE)); /* add -0.5 shift to make signal between -0.5 and 0.5 */
	d_ff_val += (int32_t)analog_offsets.d_FF; /* Centered around D_FF_OFFSET */

	if(autozero_state != AUTOZERO_IN_PROGRESS){

		DAC_write_i_SP((uint32_t)i_sp_val);
		EPWM1->CMPDAT[3] = (uint32_t)d_ff_val;/* d_FF DAC, DAC0 pin */

	}

}

void ADC_calc_averages(void){

	static uint32_t v_AC_accumulator = 0;
	static uint32_t i_PV_accumulator = 0;
	static uint32_t v_Mid_accumulator = 0;
	static uint32_t i = NB_SAMPLES_LONG_TERM_AVG;


	if(i){ /* If countdown isn't finished (not zero)*/

		v_AC_accumulator += ADC_raw_val[V_AC_CHANNEL];
		i_PV_accumulator += ADC_raw_val[I_PV_CHANNEL];
		v_Mid_accumulator += ADC_raw_val[V_MID_CHANNEL];
		i--;

	}else{ /* Else countdown is finished*/

		i = NB_SAMPLES_LONG_TERM_AVG; // reset i
		analog_avgs.v_AC = v_AC_accumulator / NB_SAMPLES_LONG_TERM_AVG; // export the content of the accumulator div. by the number of samples to get avg.
		v_AC_accumulator = 0; // reset the accumulator
		analog_avgs.i_PV = i_PV_accumulator / NB_SAMPLES_LONG_TERM_AVG;
		i_PV_accumulator = 0;
		analog_avgs.v_Mid = v_Mid_accumulator / NB_SAMPLES_LONG_TERM_AVG;
		v_Mid_accumulator = 0;

	}

}

void DAC_write_i_SP(uint32_t value){
#ifdef PWM_DAC
		EPWM1->CMPDAT[2] = value;/* i_SP DAC, DAC1 pin */
		#else
		DAC_WRITE_DATA(DAC1, 0, value);
		 DAC_START_CONV(DAC1);
		#endif

}


void analog_autozero(void){

//TODO: Add digital duty cycle feedback to be able to zero d_FF output as well. Analog duty cycle feedback would become also obsolete.

/**************State machine for autozero conditions**************************************************/

	static uint16_t guess;
	static int8_t bit_position;

	/* For the autozero to take place, some other function must bring the voltage high enough,
	 *  open the relays, and set the state to AUTOZERO_WAIT_FOR_CONDITIONS */
	bool conditions_ok = analog_check_autozero_conditions_ok();

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

				autozero_state = AUTOZERO_IN_PROGRESS;// Go to next state

				// AUTOZERO_IN_PROGRESS state entry code
				bit_position = DAC_RES_BITS - 1;
				guess = ((uint16_t)DAC_RES_COUNT)>>1; /* Initial guess, half the full DAC range */
				DAC_write_i_SP((uint32_t)guess);
				PA5 = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/

			}
			//
			break;

		case AUTOZERO_IN_PROGRESS:

			if(bit_position >= 0){ /* If there are bit positions left to evaluate, that is, bit position is not negative*/

				if(PA5){/* If compensator reset is active*/
					PA5 = false; /* remove the compensator reset (it was active for one cycle), and wait for the next execution, to give time to the compensator to saturate*/
				}else{
					if(ADC_raw_val[D_COMP_CHANNEL] > ((uint16_t)(ADC_RES_COUNT - COUNT_MARGIN_FOR_AUTOZERO))){/* If d_COMP saturated high*/
						/* That means the guess was too high */
						/* Put the bit back to zero in the guess*/
						guess &= ~(1 << bit_position);
						bit_position--;/* Go to next bit */

						if(bit_position >= 0){/* If there are bit positions left to evaluate, that is, bit position is not negative*/
							/* Prepare the next guess*/
							guess |= (1 << bit_position);/* set bit to 1 in guess at new bit position*/
							DAC_write_i_SP((uint32_t)guess);/* Write the guess to the DAC*/
							PA5 = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/
						}


					}else if(ADC_raw_val[D_COMP_CHANNEL] < ((uint16_t)COUNT_MARGIN_FOR_AUTOZERO)){/* If d_COMP saturated low*/
						/* That means the guess was too low */
						/* Keep the bit at 1 in the guess*/
						bit_position--;/* Go to next bit */

						if(bit_position >= 0){/* If there are bit positions left to evaluate, that is, bit position is not negative*/
							/* Prepare the next guess*/
							guess |= (1 << bit_position); /* set bit to 1 in guess at new bit position*/
							DAC_write_i_SP((uint32_t)guess); /* Write the guess to the DAC*/
							PA5 = true; /* Put the compensator in the reset state and leave it to reset for the next cycle*/
						}



					}else{/**/
						// Else do nothing and wait for next cycle, leave some time for the compensator to saturate
					}
				}

			}else{/* Else there aren't any bit positions left to try, the autozero is finished*/
				autozero_state = AUTOZERO_DONE;
				// AUTOZERO_IN_PROGRESS state exit code here
				analog_offsets.i_SP = guess; // Save the value found
				analog_offsets.v_AC = analog_avgs.v_AC; /* Zero the average voltage*/
				analog_offsets.i_PV = analog_avgs.i_PV; /* Zero the input current*/
			}

			//
			break;

		case AUTOZERO_DONE:
			//
			break;

		default: // If un-handled state, that's an error
			autozero_state = AUTOZERO_STANDBY; //Reset
	}

}

bool analog_check_autozero_conditions_ok(void){
	/* Voltages on the busses high enough that no current is drawn on the xformer and the AC relay and precharge are open (high) */
	return (analog_in.V_DC_plus > VBUS_MIN_FOR_AUTOZERO) && (analog_in.V_DC_minus < -VBUS_MIN_FOR_AUTOZERO) && PC2 && PC4;
}
