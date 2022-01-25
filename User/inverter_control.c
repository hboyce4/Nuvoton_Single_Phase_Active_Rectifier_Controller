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

inverter_state_variables_t inverter = {.I_D = 0};
inverter_state_safety_t inverter_safety;
inverter_state_setpoints_t inverter_setpoints = {.V_DC_total_setpoint = VBUS_TOTAL_DEFAULT,
												 .V_DC_diff_setpoint = VBUS_DIFF_DEFAULT};
//inverter_limits_t inverter_limits;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void init_inverter_control(void){

    /*Using BPWM1, */
    /* Begin to output the carrier waveform for the analog hardware PWModulator on PA.6 (pin 16 on the M481LIDAE) */
	BPWM_ConfigOutputChannel(BPWM1, 3, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480. Use the defined carrier freq, with 50% duty cycle */
	BPWM1->POEN |= BPWM_POEN_POEN3_Msk; /* Enable CH5 (set 1 at bit position 5)*/
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

	EPWM1->PERIOD[2] = (uint16_t)RES_11BIT; /* 192MHz/2048 = 93.75 kHz*/
	EPWM1->PERIOD[3] = (uint16_t)RES_11BIT;

	EPWM1->CMPDAT[2] = (uint16_t)(RES_11BIT/2);/* Some initial value*/
	EPWM1->CMPDAT[3] = (uint16_t)(RES_11BIT/2);

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

void inverter_control_main(void){


	inverter_check_i_sync();
	inverter_safety_fast();

	/* Calculate the current (i) setpoint */
	inverter_calc_I_D();
	inverter_calc_I_balance();
	//saturate();
	inverter.i_SP = (inverter.I_D * PLL.b_beta) + inverter.I_balance ;

	/* Calculate the duty cycle feedforward value */
	float d_feedforward;
	if(inverter.V_DC_total == 0){ /* If we were to run into a division by zero */
		d_feedforward = 0.5; /* don't do the calculation and just pick 50% */
	}else{
		d_feedforward = (inverter.v_AC-inverter.V_DC_minus)/inverter.V_DC_total;

		/* Saturate between 0 and 1 (0% and 100%)*/
		if(d_feedforward < 0){
			d_feedforward = 0;
		}else if(d_feedforward > 1){
			d_feedforward = 1;
		}
	}

	inverter.d_feedforward = d_feedforward;


}

void inverter_safety_fast(void){

}

void inverter_calc_I_D(void){

	static float V_DC_total_filtered;

	V_DC_total_filtered = V_DC_total_filtered + (((inverter.V_DC_total-V_DC_total_filtered)*T_CALC)/VBUS_TOTAL_LPF_TAU);

	float err_V_DC_total = V_DC_total_filtered - inverter_setpoints.V_DC_total_setpoint;

	err_V_DC_total *= VBUS_TOTAL_KP;

	/* Saturate */
	if (err_V_DC_total > I_D_MAX){
		err_V_DC_total = I_D_MAX;
	}else if (err_V_DC_total < -I_D_MAX){
		err_V_DC_total = -I_D_MAX;
	}

	inverter.I_D = err_V_DC_total;

}

void inverter_calc_I_balance(void){


	static float V_DC_diff_filtered;

		V_DC_diff_filtered = V_DC_diff_filtered + (((inverter.V_DC_diff-V_DC_diff_filtered)*T_CALC)/VBUS_DIFF_LPF_TAU);

		float err_V_DC_diff = V_DC_diff_filtered - inverter_setpoints.V_DC_diff_setpoint;

		err_V_DC_diff *= VBUS_DIFF_KP;

		/* Saturate */
		if (err_V_DC_diff > I_BALANCE_MAX){
			err_V_DC_diff = I_BALANCE_MAX;
		}else if (err_V_DC_diff < -I_BALANCE_MAX){
			err_V_DC_diff = -I_BALANCE_MAX;
		}

		inverter.I_balance = err_V_DC_diff;

}

void inverter_check_i_sync(void){

	static uint16_t set_count, reset_count;

	if((inverter.i_PV > inverter.i_SP-I_SYNC_TOL) && (inverter.i_PV < inverter.i_SP+I_SYNC_TOL)){ /* If the input voltage is within tolerance of the estimated voltage*/
		reset_count = 0;
		set_count++;
	}else{/* Else it's out of tolerance*/
		reset_count++;
		set_count = 0;
	}


	if(reset_count >= I_SYNC_COUNT_FOR_RESET){
		reset_count = I_SYNC_COUNT_FOR_RESET;
		inverter_safety.i_sync = false;
	}else if(set_count >= I_SYNC_COUNT_FOR_SET){
		set_count = I_SYNC_COUNT_FOR_SET;
		inverter_safety.i_sync = true;
	}

}
