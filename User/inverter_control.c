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
inverter_faults_t inverter_faults;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/



void inverter_control_main(void){ // Service the inverter. Needs up-to-date PLL and analog input values.

	inverter_check_safety_operational_status();

	/* Calculate the current (i) setpoint */
	inverter_calc_I_D();
	inverter_calc_I_balance();
	//saturate();
	inverter.i_SP = (inverter.I_D * PLL.b_beta) + inverter.I_balance ; /* The current setpoint is the sum of the "direct" (iD) current and the balancing current*/
	// inverter.I_D is a continuous ("DC") value, PLL.b_beta is a sine wave.  inverter.I_balance is a proportionately small "DC" value.

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

void inverter_check_safety_operational_status(void){

	inverter_check_i_sync();
	inverter_check_limits();

	inverter_calc_state();





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

void inverter_check_limits(void){

	/************* VBUS+ Overvoltage **************************************************************************************/
	if(inverter.V_DC_plus > OV_LIMIT){ /* If VBUS+ is over the overvoltage limit*/
		inverter_safety.OV_V_DC_plus = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_plus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_plus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS+ Overvoltage end **********************************************************************************/

	/************* VBUS- Overvoltage **************************************************************************************/
	if(inverter.V_DC_minus < -OV_LIMIT){ /* If VBUS- exceeds the overvoltage limit*/
		inverter_safety.OV_V_DC_minus = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_minus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_minus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS- Overvoltage end ***********************************************************************************/

	/************* VBUS+ Undervoltage **************************************************************************************/
	if(inverter.V_DC_plus < UV_LIMIT){ /* If VBUS+ is under the undervoltage limit*/
		inverter_safety.UV_V_DC_plus = TRUE; /* There is currently an undervoltage condition */
		inverter_faults.UV_V_DC_plus_fault = TRUE; /* Memorize the fault */

		if(inverter.V_DC_plus < UV2_LIMIT){ /* If VBUS+ is under the second undervoltage limit*/
			inverter_safety.UV2_V_DC_plus = TRUE; /* There is currently an undervoltage 2 condition */
			inverter_faults.UV2_V_DC_plus_fault = TRUE; /* Memorize the fault */
		}else{
			inverter_safety.UV2_V_DC_plus = FALSE; /* There is not currently an undervoltage 2 condition */
		}

	}else{
		inverter_safety.UV_V_DC_plus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS+ Undervoltage end **********************************************************************************/

	/************* VBUS- Undervoltage **************************************************************************************/
	if(inverter.V_DC_minus > -UV_LIMIT){ /* If VBUS- exceeds the undervoltage limit*/
		inverter_safety.UV_V_DC_minus = TRUE; /* There is currently an undervoltage condition */
		inverter_faults.UV_V_DC_minus_fault = TRUE; /* Memorize the fault */

		if(inverter.V_DC_minus > -UV2_LIMIT){ /* If VBUS- exceeds the second undervoltage limit*/
			inverter_safety.UV2_V_DC_minus = TRUE; /* There is currently an undervoltage 2 condition */
			inverter_faults.UV2_V_DC_minus_fault = TRUE; /* Memorize the fault */
		}else{
			inverter_safety.UV2_V_DC_minus = FALSE; /* There is not currently an undervoltage condition */
		}
	}else{
		inverter_safety.UV_V_DC_minus = FALSE; /* There is not currently an undervoltage condition */
	}
	/************* VBUS- Undervoltage end ***********************************************************************************/

	/************* VBUS imbalance **************************************************************************************/
	if(abs(inverter.V_DC_diff) > DIFF_LIMIT){ /* If VBUS+ is over the overvoltage limit*/
		inverter_safety.OV_V_DC_diff = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_diff_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_diff = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS Imbalance end **********************************************************************************/
	;

}

void inverter_calc_state(void){

/**************State machine for startup and operational conditions**************************************************/

	static uint32_t precharge_timer = 0;
	static uint32_t charge_timer = 0;


	if((!PLL.sync)||(!inverter_setpoints.inverter_active)){ // If no PLL sync or inverter set to off by user
		inverter_safety.operating_state = OPER_OFF; // switch to the OFF state immediately
	}

	switch (inverter_safety.operating_state){

	case OPER_OFF:

		//Set all relays off here (signals are active low)
		PC2 = TRUE; // Precharge opto
		PC4 = TRUE; // Relay

		if(PLL.sync && inverter_setpoints.inverter_active){ // If we have PLL sync and the inverter is set to ON



			inverter_safety.operating_state = OPER_PRECHARGE; // Go into precharge state

			// PRECHARGE state entry code here
			precharge_timer = PRECHARGE_TIMEOUT;// Set the precharge timer
			// PRECHARGE state entry code end

		}
		break;


	case OPER_PRECHARGE:

		// Turn on only precharge opto here (signals are active low)
		PC2 = FALSE; // Precharge opto
		PC4 = TRUE; // Relay


		/* Timeout here*/
		if(precharge_timer){ /* If time left on the timer*/
			precharge_timer--; /* Decrease timer*/
		}else{ /* Else timer has elapsed */
			// PRECHARGE state exit code would go here

			inverter_safety.operating_state = OPER_OFF; // switch to the OFF state immediately

			// OFF state entry code would go here
		}

		/* Bus voltages high enough, phase angle is good*/
		if((!inverter_safety.UV2_V_DC_plus)&&(!inverter_safety.UV2_V_DC_minus)&&(PLL.theta_est > THETA_MIN_RELAY_CLOSE)&&(PLL.theta_est < THETA_MAX_RELAY_CLOSE)){

			// PRECHARGE state exit code here
			precharge_timer = 0;
			// PRECHARGE state exit code end

			inverter_safety.operating_state = OPER_WAIT_FOR_CLOSE; // go to the WAIT_FOR_CLOSE state

			// WAIT_FOR_CLOSE state entry code here

		}
		break;


	case OPER_WAIT_FOR_CLOSE:

		//**** Turn on precharge opto AND AC relay here (signals are active low)
		PC2 = FALSE; // Precharge opto
		//PC4 = FALSE; // Relay


		if(PLL.theta_est > SLIGHTLY_LESS_THAN_2_PI){ /* If the end of the cycle and the zero crossing is near*/
			inverter_safety.operating_state = OPER_DWELL; // go to the dwell state;
		}

		break;

	case OPER_DWELL:

		//**** Turn on precharge opto AND AC relay here (signals are active low)
		PC2 = FALSE; // Precharge opto
		//PC4 = FALSE; // Relay

		if((PLL.theta_est < SLIGHTLY_LESS_THAN_2_PI)&&(PLL.theta_est > THETA_MIN_GARANTEED_CLOSE)){/*If we are some time after the 0-crossing, we are sure the relay has closed and we can turn off the precharge*/
			inverter_safety.operating_state = OPER_CHARGE; // go to the CHARGE state;

			// CHARGE state entry code here
			charge_timer = CHARGE_TIMEOUT;// Set the precharge timer
			// CHARGE state entry code end

		}
		break;

	case OPER_CHARGE:

		//**** Turn on only relay here (signals are active low)
		PC2 = TRUE; // Precharge opto
		//PC4 = FALSE; // Relay

		/* Timeout here*/
		if(charge_timer){ /* If time left on the timer*/
			charge_timer--; /* Decrease timer*/
		}else{ /* Else timer has elapsed */

			// CHARGE state exit code would go here

			inverter_safety.operating_state = OPER_OFF; // switch to the OFF state immediately

			// OFF state entry code would go here
		}

		if((!inverter_safety.UV_V_DC_minus) && (!inverter_safety.UV_V_DC_plus)){/* If no undervoltage condition is present*/

			inverter_safety.operating_state = OPER_AC_ON; // go to the AC_ON state;

			// CHARGE state exit code here
			precharge_timer = 0;
			// CHARGE state exit code end
		};
		break;

	case OPER_AC_ON:

		//**** Turn on only relay here (signals are active low)
		PC2 = TRUE; // Precharge opto
		//PC4 = FALSE; // Relay

		// No exit condition other than user OFF or PLL desync;
		break;

	default: // If unknown state
		inverter_safety.operating_state = OPER_OFF; // switch to the OFF state immediately

	}
/**************End of state machine for startup and operational conditions**************************************************/


}
