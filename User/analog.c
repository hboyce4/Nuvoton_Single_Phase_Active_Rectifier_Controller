/*
 * analog.c
 *
 *  Created on: Feb 6, 2021
 *      Author: Hugo Boyce
 */

#include "analog.h"
#include "NuMicro.h"
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile uint32_t ADC_raw_val[EADC_LAST_CHANNEL+1];
volatile uint16_t ADC_acq_buff[EADC_LAST_CHANNEL+1];
volatile uint8_t ADC_acq_count;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
// TODO: Add conversion to Celsius  for the temperature channels

void ADC_run_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk|EADC_CALCTL_CALSEL_Msk;/* Set CALSTART to start calibration. Set CALSEL so it doesn't try to load user calibration word*/
	/* EADCDIV must be zero to run calibration */
}

void ADC_process_interrupt(void){
	/* Very high frequency interrupt. Keep very light!!! */
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk); /* Clear the A/D ADINT0 interrupt flag */
	uint8_t channel;
	for (channel = EADC_FIRST_CHANNEL; channel < EADC_LAST_CHANNEL; channel++) {
		/* Acquire latest data from ADC, filtering out status bits*/
		ADC_acq_buff[channel] += (uint16_t) (EADC->DAT[channel]); /* Only keep the lowest 16 bits of the register*/
	}
	PWM_acc_count += BPWM_GET_CAPTURE_FALLING_DATA(BPWM0, 1); /* timer count*/
	;
	ADC_acq_count--; /* Decrease the number of acquisitions left to make*/
	if (!ADC_acq_count) {
		/* If zero acquisitions left to do */
		NVIC_DisableIRQ(EADC00_IRQn); /* Stop the interrupt */
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

void DAC_write_d_FF(uint32_t value){
#ifdef PWM_DAC
		EPWM1->CMPDAT[3] = value;/* d_FF DAC, DAC0 pin */
		#else
		/* Do nothing*/
		#endif

}

uint32_t DAC_read_d_FF(void){
#ifdef PWM_DAC
		return EPWM1->CMPDAT[3];/* d_FF DAC, DAC0 pin */
		#else
		/* Do nothing*/
		#endif

}

