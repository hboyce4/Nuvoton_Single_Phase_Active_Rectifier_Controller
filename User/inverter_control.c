/*
 * inverter_control.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

inverter_state_variables_t inverter_state_variables;
inverter_state_safety_t inverter_state_safety;
inverter_setpoints_t inverter_setpoints;
inverter_limits_t inverter_limits;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void init_inverter_control(void){

    /*Using BPWM1, */
    /* Begin to output the carrier waveform for the analog hardware PWModulator on PA.15 (pin 100 on the M487JIDAE) */
	BPWM_ConfigOutputChannel(BPWM1, 5, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480. Use the defined carrier freq, with 50% duty cycle */
	BPWM1->POEN |= BPWM_POEN_POEN5_Msk; /* Enable CH5 (set 1 at bit position 5)*/
	BPWM1->CNTEN = BPWM_CNTEN_CNTEN0_Msk;

#ifdef PWM_DAC
	 /*Using EPWM1 CH2 & CH3 */
	/* Start PWM generation for the PWM DACs */
	/* set EPWM to up counter type(edge aligned) */
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTTYPE2_Msk))|(EPWM_UP_COUNTER << EPWM_CTL1_CNTTYPE2_Pos);
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTTYPE3_Msk))|(EPWM_UP_COUNTER << EPWM_CTL1_CNTTYPE3_Pos);
	/* set EPWM to auto-reload by clearing CNTMODE bits*/
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTMODE2_Msk));
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTMODE3_Msk));

	EPWM1->CLKPSC[1] = 0;/* CLKPSC[1] is EPWM_CLKPSC2_3 which is the prescaler for channels 2 & 3*/

	EPWM1->PERIOD[2] = (uint16_t)RES_12BIT;
	EPWM1->PERIOD[3] = (uint16_t)RES_12BIT;

	EPWM1->CMPDAT[2] = 2048;/* Some initial value*/
	EPWM1->CMPDAT[3] = 2048;

	EPWM1->WGCTL0 = (EPWM1->WGCTL0 & ~(EPWM_WGCTL0_PRDPCTL2_Msk | EPWM_WGCTL0_ZPCTL2_Msk)) | ((uint32_t)EPWM_OUTPUT_HIGH << EPWM_WGCTL0_ZPCTL2_Pos);
	EPWM1->WGCTL0 = (EPWM1->WGCTL0 & ~(EPWM_WGCTL0_PRDPCTL3_Msk | EPWM_WGCTL0_ZPCTL3_Msk)) | ((uint32_t)EPWM_OUTPUT_HIGH << EPWM_WGCTL0_ZPCTL3_Pos);

	EPWM1->WGCTL1 = (EPWM1->WGCTL1 & ~(EPWM_WGCTL1_CMPDCTL2_Msk | EPWM_WGCTL1_CMPUCTL2_Msk)) | ((uint32_t)EPWM_OUTPUT_LOW << EPWM_WGCTL1_CMPUCTL2_Pos);
	EPWM1->WGCTL1 = (EPWM1->WGCTL1 & ~(EPWM_WGCTL1_CMPDCTL3_Msk | EPWM_WGCTL1_CMPUCTL3_Msk)) | ((uint32_t)EPWM_OUTPUT_LOW << EPWM_WGCTL1_CMPUCTL3_Pos);
	//EPWM_ConfigOutputChannel(EPWM1, 5, PWM_CARRIER_FREQ, 50)


	EPWM1->POEN |= EPWM_POEN_POEN2_Msk; /* Enable CH2 (set 1 at bit position 2)*/
	EPWM1->POEN |= EPWM_POEN_POEN3_Msk; /* Enable CH3 (set 1 at bit position 3)*/

	EPWM1->CNTEN |= EPWM_CNTEN_CNTEN2_Msk;
	EPWM1->CNTEN |= EPWM_CNTEN_CNTEN3_Msk;

#endif
	delay_ms(10); /* Give the analog modulator some time to stabilize */





	/* Start timer 1. Triggers the control loop interrupt */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, F_CALC); /* Set the interrupt frequency as F_CALC*/
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_SetPriority(TMR1_IRQn, TMR1_INT_PRIORITY);
    TIMER_Start(TIMER1);

}
