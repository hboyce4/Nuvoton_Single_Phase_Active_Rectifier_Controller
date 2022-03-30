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
inverter_state_setpoints_t inverter_setpoints = {.inverter_active = 0,
												 .V_DC_total_setpoint = VBUS_TOTAL_DEFAULT,
												 .V_DC_diff_setpoint = VBUS_DIFF_DEFAULT};
//inverter_limits_t inverter_limits;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/



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


	if(!PLL.sync){ // If no PLL sync
		inverter_safety.operating_state = OFF; // switch to the OFF state immediately
	}

	switch (inverter_safety.operating_state){

	case OFF:

		/*****Set all relays off here*******/

		if(PLL.sync && inverter_setpoints.inverter_active){ // If we have PLL sync and the inverter is set to ON
			inverter_safety.operating_state = PRECHARGE; // Go into precharge mode
		}
		break;


	case PRECHARGE:


		/**** Turn on AC precharge relay here****/

		if(/* Bus voltages hig enough, phase angle is good*/){
			inverter_safety.operating_state = WAIT_FOR_CLOSE; // go to the WAIT_FOR_CLOSE state
		}

		break;


	case WAIT_FOR_CLOSE:

		/**** Turn on both AC relays here****/

		;
		break;

	case DWELL:
		;
		break;

	case RELAY_ON:
		;
		break;

	}



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
