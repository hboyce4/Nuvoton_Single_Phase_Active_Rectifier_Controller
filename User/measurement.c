/*
 * measurement.c
 *
 *  Created on: Jan. 13, 2024
 *      Author: Hugo Boyce
 */
#include "measurement.h"
#include "analog.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile measurement_inputs_t measurements_in; /* directly measured values */

volatile measurement_averages_t measurement_avgs;

volatile measurement_offsets_t measurement_offsets = {.v_AC = V_AC_OFFSET,
											.i_PV = I_PV_OFFSET,
											.i_SP = I_SP_OFFSET,
											.d_FF = D_FF_OFFSET};


/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void Measurement_process_oversampling(void){

	/* Divide,copy and reset */
	uint8_t channel;
	for(channel = EADC_FIRST_CHANNEL; channel <= EADC_LAST_CHANNEL;channel++){/* For every channel */

		/* this step could be skipped by integrating this calculation in the float conversion step*/
		ADC_raw_val[channel]= (ADC_acq_buff[channel] >> EADC_SHIFT_FOR_OVERSAMPLING_DIVISION); /* Copy the acquisition buffer, dividing by the number of oversamples. */

		ADC_acq_buff[channel] = 0;


	}

	PWM_raw_count = (PWM_acc_count >> EADC_SHIFT_FOR_OVERSAMPLING_DIVISION);
	/* Check saturation here or somewhere else*/
	PWM_acc_count = 0;

	/* Start ADC acquisition batch*/
	ADC_acq_count = EADC_OVERSAMPLING_NUMBER;

	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	NVIC_EnableIRQ(EADC00_IRQn);

}



void Measurement_convert_to_float(void){

	/* Vbus plus */
	measurements_in.V_DC_plus = ((float)ADC_raw_val[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*VBUS_PLUS_GAIN))-VBUS_PLUS_OFFSET;
	/* Vbus minus */
	measurements_in.V_DC_minus = -(((float)ADC_raw_val[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*VBUS_MINUS_GAIN)))-VBUS_MINUS_OFFSET;
	/* Vbus total */
	measurements_in.V_DC_total = measurements_in.V_DC_plus - measurements_in.V_DC_minus;
	/* Vbus diff */
	measurements_in.V_DC_diff = measurements_in.V_DC_plus + measurements_in.V_DC_minus;
	/* V AC */
	measurements_in.v_AC = ((float)ADC_raw_val[V_AC_CHANNEL] - (float)measurement_offsets.v_AC)*(VREF_VOLTAGE/(ADC_RES_COUNT*V_AC_GAIN));
	/* V AC normalized to 1 */
	measurements_in.v_AC_n = measurements_in.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*M_SQRT2)); /* needs math.h */
	/* Current process value (actual amperage) */
	measurements_in.i_PV = ((float)ADC_raw_val[I_PV_CHANNEL] - (float)measurement_offsets.i_PV)*(VREF_VOLTAGE/(ADC_RES_COUNT*I_PV_GAIN));


	measurements_in.d_sat = (bool)((BPWM0->INTSTS) & BPWM_INTSTS_PIF0_Msk) >> BPWM_INTSTS_PIF0_Pos;/* Check if duty cycle is saturated*/
	/*by checking if the timer has reached the compare point. If the timer reaches the compare point, its because there were no edges */
	/* to re-trigger it, meaning its stuck high or low.*/

	if(measurements_in.d_sat){/* If saturated*/
		 BPWM0->INTSTS = BPWM_INTSTS_PIF0_Msk; // Clear the flag by writing one to it
		 if(PA10){ /* If pin is high*/
			 measurements_in.d = 1.0; /* the duty cycle is saturated high (100%)*/
		 }else{
			 measurements_in.d = 0.0; /* the duty cycle is saturated low (0%)*/
		 }

	}else{
		measurements_in.d = (float)(PWM_raw_count)/480.0f; //TODO: Remove magic number. Equal to CNR of BPWM1
	}


}



void Measurement_convert_to_int_and_write(void){

	int32_t i_sp_val, d_ff_val;

	/* Convert the current setpoint from float to int*/
	i_sp_val = 	(int32_t)(inverter.i_SP*I_SP_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE));
	i_sp_val += (int32_t)measurement_offsets.i_SP; /* signal centered around I_SP_OFFSET */

	/* Convert the duty cycle feedforward value from int to float */
	if(g_d_ff_zero_state){
		d_ff_val = 0;
	}else{
		d_ff_val = (int32_t)((inverter.d_feedforward-0.5)*D_FF_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE)); /* add -0.5 shift to make signal between -0.5 and 0.5 */
	}
	d_ff_val += (int32_t)measurement_offsets.d_FF; /* Centered around D_FF_OFFSET */

	if(autozero_state != AUTOZERO_I_IN_PROGRESS){

		DAC_write_i_SP((uint32_t)i_sp_val);
		EPWM1->CMPDAT[3] = (uint32_t)d_ff_val;/* d_FF DAC, DAC0 pin */

	}

}

void Measurement_calc_averages(void){

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
		measurement_avgs.v_AC = v_AC_accumulator / NB_SAMPLES_LONG_TERM_AVG; // export the content of the accumulator div. by the number of samples to get avg.
		v_AC_accumulator = 0; // reset the accumulator
		measurement_avgs.i_PV = i_PV_accumulator / NB_SAMPLES_LONG_TERM_AVG;
		i_PV_accumulator = 0;
		measurement_avgs.v_Mid = v_Mid_accumulator / NB_SAMPLES_LONG_TERM_AVG;
		v_Mid_accumulator = 0;

	}

}
