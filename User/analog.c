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
volatile int32_t ADC_raw_val[EADC_TOTAL_CHANNELS];
volatile uint16_t ADC_acq_buff[EADC_TOTAL_CHANNELS];
volatile uint8_t ADC_acq_count;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/


void run_ADC_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk|EADC_CALCTL_CALSEL_Msk;/* Set CALSTART to start calibration. Set CALSEL so it doesn't try to load user calibration word*/
	/* EADCDIV must be zero to run calibration */
}

void process_ADC(void){

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



void convert_to_float(void){

	/* Vbus plus */
	analog_in.V_DC_plus = ((float)ADC_raw_val[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*VBUS_PLUS_GAIN))-VBUS_PLUS_OFFSET;
	/* Vbus minus */
	analog_in.V_DC_minus = -(((float)ADC_raw_val[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*VBUS_MINUS_GAIN)))-VBUS_MINUS_OFFSET;
	/* Vbus total */
	analog_in.V_DC_total = analog_in.V_DC_plus - analog_in.V_DC_minus;
	/* Vbus diff */
	analog_in.V_DC_diff = analog_in.V_DC_plus + analog_in.V_DC_minus;
	/* V AC */
	analog_in.v_AC = ((float)ADC_raw_val[V_AC_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*V_AC_GAIN))-V_AC_OFFSET;
	/* V AC normalized to 1 */
	analog_in.v_AC_n = analog_in.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*M_SQRT2)); /* needs math.h */
	/* Current process value (actual amperage) */
	analog_in.i_PV = ((float)ADC_raw_val[I_PV_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*I_PV_GAIN))-I_PV_OFFSET;
}



void convert_to_int_write_analog(void){

	int32_t i_sp_val, d_ff_val;

	/* Convert the current setpoint from float to int*/
	i_sp_val = 	(int32_t)(inverter.i_SP*I_SP_GAIN*(RES_11BIT/VREF_VOLTAGE));
	i_sp_val += I_SP_OFFSET; /* signal centered around I_SP_OFFSET */

	/* Convert the duty cycle feedforward value from int to float */
	d_ff_val = (int32_t)((inverter.d_feedforward-0.5)*D_FF_GAIN*(RES_11BIT/VREF_VOLTAGE)); /* add -0.5 shift to make signal between -0.5 and 0.5 */
	d_ff_val += D_FF_OFFSET; /* Centered around D_FF_OFFSET */

#ifdef PWM_DAC

	EPWM1->CMPDAT[2] = (uint32_t)i_sp_val;/* DAC1 pin */
	EPWM1->CMPDAT[3] = (uint32_t)d_ff_val;/* DAC0 pin */
#else
	 DAC_WRITE_DATA(DAC1, 0, i_sp_val);
	 DAC_START_CONV(DAC1);
#endif


}

