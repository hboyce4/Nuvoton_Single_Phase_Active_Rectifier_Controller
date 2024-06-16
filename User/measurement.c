/*
 * measurement.c
 *
 *  Created on: Jan. 13, 2024
 *      Author: Hugo Boyce
 */
#include "measurement.h"
#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Local Macros           				                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

#define OMEGA_SHORT_TERM_AVERAGE 6.28f //[rad/s] Cutoff frequency for the low pass filter for the long term averages
#define ABS_AVG_TO_RMS 1.11072f //Factor to transfrom from absolute average to RMS *for a perfect sine wave* Equalt to (pi/2)/sqrt(2)

#define NTC_R_ZERO 10000.0f //Resistance @ /normal" temperature
#define NTC_T_ZERO 298.15f /* [K] Equals 25 degree C. Temperature at which the NTC's R_Zero is measured. */
#define NTC_BETA 3400.0f // Beta coefficient of the thermistors
#define NTC_BIAS_RESISTOR 6200.0f // [Ohms] Value of the NTC biasing resistor

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

		/* DONE: this step could be skipped by integrating this calculation in the float conversion step*/
		// Not being divided to cancel the oversampling anymore
		ADC_raw_val[channel]= (uint32_t)ADC_acq_buff[channel]; /* Copy the acquisition buffer, dividing by the number of oversamples. */

		ADC_acq_buff[channel] = 0;


	}

	// Not being divided to cancel the oversampling anymore
	PWM_raw_count = (PWM_acc_count);
	/* Check saturation here or somewhere else*/
	PWM_acc_count = 0;

	/* Start ADC acquisition batch*/
	ADC_acq_count = EADC_OVERSAMPLING;

	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	NVIC_EnableIRQ(EADC00_IRQn);

}



void Measurement_convert_to_float(void){

	/* Vbus plus */
	measurements_in.V_DC_plus = ((float)ADC_raw_val[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*EADC_OVERSAMPLING*VBUS_PLUS_GAIN))-VBUS_PLUS_OFFSET;
	/* Vbus minus */
	measurements_in.V_DC_minus = -(((float)ADC_raw_val[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES_COUNT*EADC_OVERSAMPLING*VBUS_MINUS_GAIN)))-VBUS_MINUS_OFFSET;
	/* Vbus total */
	measurements_in.V_DC_total = measurements_in.V_DC_plus - measurements_in.V_DC_minus;
	/* Vbus diff */
	measurements_in.V_DC_diff = measurements_in.V_DC_plus + measurements_in.V_DC_minus;
	/* V AC */
	measurements_in.v_AC = ((float)ADC_raw_val[V_AC_CHANNEL] - (float)measurement_offsets.v_AC)*(VREF_VOLTAGE/(ADC_RES_COUNT*EADC_OVERSAMPLING*V_AC_GAIN));
	/* V AC normalized to 1 */
	measurements_in.v_AC_n = measurements_in.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*((float)M_SQRT2))); /* needs math.h */
	/* Current process value (actual amperage) */
	measurements_in.i_PV = ((float)ADC_raw_val[I_PV_CHANNEL] - (float)measurement_offsets.i_PV)*(VREF_VOLTAGE/(ADC_RES_COUNT*EADC_OVERSAMPLING*I_PV_GAIN));


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
		measurements_in.d = (float)(PWM_raw_count)/((float)BPWM_GET_CNR(BPWM1,0)*EADC_OVERSAMPLING);
	}


}



void Measurement_convert_to_int_and_write(void){

	int32_t i_sp_val, d_ff_val;

	/* Convert the current setpoint from float to int*/
	i_sp_val = 	(int32_t)(inverter.i_SP*I_SP_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE));
	i_sp_val += (int32_t)measurement_offsets.i_SP; /* signal centered around I_SP_OFFSET */

	/* Convert the duty cycle feedforward value from int to float */
	if(/*g_d_ff_zero_state*/false){
		d_ff_val = 0;
	}else{
		d_ff_val = (int32_t)((inverter.d_feedforward-0.5f)*D_FF_GAIN*(DAC_RES_COUNT/VREF_VOLTAGE)); /* add -0.5 shift to make signal between -0.5 and 0.5 */
	}
	d_ff_val += (int32_t)measurement_offsets.d_FF; /* Centered around D_FF_OFFSET */

	DAC_write_i_SP((uint32_t)i_sp_val);
	DAC_write_d_FF((uint32_t)d_ff_val);/* d_FF DAC, DAC0 pin */

}

void Measurement_calc_averages_short_term(void){
	float inst_power;

	inst_power = measurements_in.v_AC * measurements_in.i_PV;

	//Forward euler
	inverter.P_AC_AVG += T_CALC * OMEGA_SHORT_TERM_AVERAGE * (inst_power - inverter.P_AC_AVG);

	static float abs_avg_V_AC, abs_avg_I_AC;

	//Forward euler
	abs_avg_V_AC += T_CALC * OMEGA_SHORT_TERM_AVERAGE * (fabsf(measurements_in.v_AC) - abs_avg_V_AC);
	abs_avg_I_AC += T_CALC * OMEGA_SHORT_TERM_AVERAGE * (fabsf(measurements_in.i_PV) - abs_avg_I_AC);

	/* Transform absolute average to RMS using a factor valid for some waveforms*/
	inverter.V_AC_RMS = abs_avg_V_AC * ABS_AVG_TO_RMS;
	inverter.I_AC_RMS = abs_avg_I_AC * ABS_AVG_TO_RMS;

}


void Measurement_calc_averages_longterm(void){ /*TODO: Replace this with a IIR low pass filter */

	//TODO: Adapt this for oversampled raw values!

	// Used to autozero offsets

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

void Measurement_convert_temperatures(void){

	/* Using the Steinhart-Hart equation as per Wikipedia, get the Kelvin value */
	float r_inf, r_NTC_inverter, r_NTC_transformer;
	r_inf = NTC_R_ZERO * expf(-NTC_BETA / NTC_T_ZERO);

	r_NTC_inverter = ((-((float)ADC_raw_val[PCB_TEMP_CHANNEL]) * NTC_BIAS_RESISTOR)/(ADC_raw_val[PCB_TEMP_CHANNEL] - EADC_OVERSAMPLING*ADC_RES_COUNT));

	r_NTC_transformer = ((-((float)ADC_raw_val[XFORMER_TEMP_CHANNEL]) * NTC_BIAS_RESISTOR)/(ADC_raw_val[XFORMER_TEMP_CHANNEL] - EADC_OVERSAMPLING*ADC_RES_COUNT));;

	measurements_in.T_inverter = NTC_BETA / logf(r_NTC_inverter/r_inf) - 273.15f; // Substract absolute zero

	measurements_in.T_transformer = NTC_BETA / logf(r_NTC_transformer/r_inf) - 273.15f; // Substract absolute zero

}
