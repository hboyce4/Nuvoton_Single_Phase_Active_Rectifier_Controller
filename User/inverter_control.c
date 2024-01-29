/*
 * inverter_control.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#include "inverter_control.h"
#include "sys.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

inverter_state_variables_t inverter = {.I_D = 0};
inverter_state_safety_t inverter_safety;
inverter_state_setpoints_t inverter_setpoints = {.contactor_close_request = 0,
												 .precharge_threshold = UV2_LIMIT,
												 .V_DC_total_setpoint = VBUS_TOTAL_DEFAULT,
												 .V_DC_diff_setpoint = VBUS_DIFF_DEFAULT};
inverter_faults_t inverter_faults;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/



void inverter_control_main(void){ // Service the inverter. Needs up-to-date PLL and analog input values.
	/* High frequency -> Use T_CALC */
	PA4 = false;
	inverter_safety.d_ff_ok = false;

	inverter_check_safety_operational_status();

	if(inverter_safety.d_ff_ok){
		PA4 = true; /* rising edge to enable the inverter*/
	}

	PA15 = !PA3;/* Make the blue LED display the status of the current latch*/

	//saturate();
	inverter.i_SP = (inverter.I_D * PLL.b_beta) + inverter.I_balance ; /* The current setpoint is the sum of the "direct" (iD) current and the balancing current*/
	// inverter.I_D is a continuous ("DC") value, PLL.b_beta is a sine wave.  inverter.I_balance is a proportionately small "DC" value.

}


void inverter_check_safety_operational_status(void){

	//Update all variables and flags
	inverter_check_PLL_sync();
	inverter_check_i_sync();
	inverter_check_voltage_limits();
	inverter_calc_d_ff();

	/* Calculate the operating state*/
	inverter_calc_state();

}

void inverter_check_PLL_sync(void){

	static uint16_t set_count, reset_count;

	if((measurements_in.v_AC_n > PLL.b_beta-PLL_SYNC_TOL) && (measurements_in.v_AC_n < PLL.b_beta+PLL_SYNC_TOL)){ /* If the input voltage is within tolerance of the estimated voltage*/
		reset_count = 0;
		set_count++;
	}else{/* Else it's out of tolerance*/
		reset_count++;
		set_count = 0;
	}


	if(reset_count >= PLL_SYNC_COUNT_FOR_RESET){
		reset_count = PLL_SYNC_COUNT_FOR_RESET; // So reset_count doesn't overflow
		inverter_safety.PLL_sync = false;
		inverter_faults.PLL_sync_fault = true; // memorize the loss of sync

	}else if(set_count >= PLL_SYNC_COUNT_FOR_SET){
		set_count = PLL_SYNC_COUNT_FOR_SET; // So set_count doesn't overflow
		inverter_safety.PLL_sync = true;
	}


}

void inverter_check_i_sync(void){

	static uint16_t set_count, reset_count;

	if((measurements_in.i_PV > inverter.i_SP-I_SYNC_TOL) && (measurements_in.i_PV < inverter.i_SP+I_SYNC_TOL)){ /* If the input voltage is within tolerance of the estimated voltage*/
		reset_count = 0;
		set_count++;
	}else{/* Else it's out of tolerance*/
		reset_count++;
		set_count = 0;
	}


	if(reset_count >= I_SYNC_COUNT_FOR_RESET){
		reset_count = I_SYNC_COUNT_FOR_RESET;
		inverter_safety.i_sync = false;
		inverter_faults.i_sync_fault = true; /* loss of current sync is a fault*/
	}else if(set_count >= I_SYNC_COUNT_FOR_SET){
		set_count = I_SYNC_COUNT_FOR_SET;
		inverter_safety.i_sync = true;
	}

}

void inverter_check_voltage_limits(void){

	/************* VBUS+ Overvoltage **************************************************************************************/
	if(measurements_in.V_DC_plus > OV_LIMIT){ /* If VBUS+ is over the overvoltage limit*/
		inverter_safety.OV_V_DC_plus = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_plus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_plus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS+ Overvoltage end **********************************************************************************/

	/************* VBUS- Overvoltage **************************************************************************************/
	if(measurements_in.V_DC_minus < -OV_LIMIT){ /* If VBUS- exceeds the overvoltage limit*/
		inverter_safety.OV_V_DC_minus = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_minus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_minus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS- Overvoltage end ***********************************************************************************/

	/************* VBUS+ Undervoltage **************************************************************************************/
	if(measurements_in.V_DC_plus < UV_LIMIT){ /* If VBUS+ is under the undervoltage limit*/
		inverter_safety.UV_V_DC_plus = TRUE; /* There is currently an undervoltage condition */
		inverter_faults.UV_V_DC_plus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.UV_V_DC_plus = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS+ Undervoltage end **********************************************************************************/

	/************* VBUS+ Undervoltage 2 **************************************************************************************/
	if(measurements_in.V_DC_plus < /*UV2_LIMIT*/inverter_setpoints.precharge_threshold){ /* If VBUS+ is under the second undervoltage limit*/
		inverter_safety.UV2_V_DC_plus = TRUE; /* There is currently an undervoltage 2 condition */
		inverter_faults.UV2_V_DC_plus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.UV2_V_DC_plus = FALSE; /* There is not currently an undervoltage 2 condition */
	}
	/************* VBUS+ Undervoltage 2 end **********************************************************************************/

	/************* VBUS- Undervoltage **************************************************************************************/
	if(measurements_in.V_DC_minus > -UV_LIMIT){ /* If VBUS- exceeds the undervoltage limit*/
		inverter_safety.UV_V_DC_minus = TRUE; /* There is currently an undervoltage condition */
		inverter_faults.UV_V_DC_minus_fault = TRUE; /* Memorize the fault */


	}else{
		inverter_safety.UV_V_DC_minus = FALSE; /* There is not currently an undervoltage condition */
	}
	/************* VBUS- Undervoltage end ***********************************************************************************/

	/************* VBUS- Undervoltage 2 **************************************************************************************/
	if(measurements_in.V_DC_minus > /*-UV2_LIMIT*/-inverter_setpoints.precharge_threshold){ /* If VBUS- exceeds the second undervoltage limit*/
		inverter_safety.UV2_V_DC_minus = TRUE; /* There is currently an undervoltage 2 condition */
		inverter_faults.UV2_V_DC_minus_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.UV2_V_DC_minus = FALSE; /* There is not currently an undervoltage condition */
	}
	/************* VBUS- Undervoltage 2 end ***********************************************************************************/


	/************* VBUS imbalance **************************************************************************************/
	if(fabs(measurements_in.V_DC_diff) > DIFF_LIMIT){ /* If VBUS+ is over the overvoltage limit*/
		inverter_safety.OV_V_DC_diff = TRUE; /* There is currently an overvoltage condition */
		inverter_faults.OV_V_DC_diff_fault = TRUE; /* Memorize the fault */
	}else{
		inverter_safety.OV_V_DC_diff = FALSE; /* There is not currently an overvoltage condition */
	}
	/************* VBUS Imbalance end **********************************************************************************/

}

void inverter_calc_d_ff(void){

	/* Calculate the duty cycle feedforward value */

	float d; /* Theoretical duty cycle*/

	if(measurements_in.V_DC_total != 0){ /* If we're not dividing by zero*/

		d = (measurements_in.v_AC-measurements_in.V_DC_minus)/measurements_in.V_DC_total; // Calc the duty cycle

		/* Saturate between 0 and 1 (0% and 100%) and make the value available for the analog output*/
		if(d < 0){
			inverter.d_feedforward = 0;
		}else if(d > 1){
			inverter.d_feedforward = 1;
		} else {
			inverter.d_feedforward = d;
		}

		if((d > D_MARGIN)&&(d < 1-D_MARGIN)){ /*If the duty cycle is within the inverter's capabilities*/
			inverter_safety.d_ff_ok = true;
		}else{
			inverter_safety.d_ff_ok = false;
		}

	}else{
		inverter.d_feedforward = 0.5; /* don't do the calculation and just pick 50% */
		inverter_safety.d_ff_ok = false;
	}
}

void inverter_calc_state(void){

/**************State machine for startup and operational conditions**************************************************/

	static uint32_t precharge_timer = 0;
	static uint32_t charge_timer = 0;


	if((!inverter_safety.PLL_sync)||(!inverter_setpoints.contactor_close_request)){ // If no PLL sync or inverter set to off by user
		inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
	}

	switch(inverter_safety.contactor_state){

		case CONTACTOR_OFF:

			//Set all relays off here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = TRUE; // Relay
			RED_LED_PIN = TRUE;// Turn OFF red LED

			if(inverter_safety.PLL_sync && inverter_setpoints.contactor_close_request){ // If we have PLL sync and the inverter is set to ON



				inverter_safety.contactor_state = CONTACTOR_AC_PRECHARGE; // Go into precharge state

				// PRECHARGE state entry code here
				precharge_timer = PRECHARGE_TIMEOUT;// Set the precharge timer
				// PRECHARGE state entry code end

			}
			break;


		case CONTACTOR_AC_PRECHARGE:

			// Turn on only precharge opto here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // Precharge opto
			AC_RELAY_PIN = TRUE; // Relay OFF
			RED_LED_PIN = TRUE;// Turn OFF red LED


			/* Timeout here*/
			if(precharge_timer){ /* If time left on the timer*/
				precharge_timer--; /* Decrease timer*/
			}else{ /* Else timer has elapsed */
				// PRECHARGE state exit code goes here

				inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
				inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling
				inverter_faults.precharge_timeout_fault = TRUE; // To save the fault event

				// OFF state entry code would go here
			}

			/* Bus voltages high enough, phase angle is good*/
			if((!inverter_safety.UV2_V_DC_plus)&&(!inverter_safety.UV2_V_DC_minus)&&(PLL.theta_est > THETA_MIN_RELAY_CLOSE)&&(PLL.theta_est < THETA_MAX_RELAY_CLOSE)){

				// PRECHARGE state exit code here
				precharge_timer = 0;
				// PRECHARGE state exit code end

				inverter_safety.contactor_state = CONTACTOR_AC_WAIT_FOR_CLOSE; // go to the WAIT_FOR_CLOSE state

				// WAIT_FOR_CLOSE state entry code here

			}
			break;


		case CONTACTOR_AC_WAIT_FOR_CLOSE:

			//**** Turn on precharge opto AND AC relay here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON
			RED_LED_PIN = FALSE;// Turn on red LED


			if(PLL.theta_est > SLIGHTLY_LESS_THAN_2_PI){ /* If the end of the cycle and the zero crossing is near*/
				inverter_safety.contactor_state = CONTACTOR_AC_DWELL; // go to the dwell state;
			}

			break;

		case CONTACTOR_AC_DWELL:

			//**** Turn on precharge opto AND AC relay here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON
			RED_LED_PIN = FALSE;// Turn on red LED

			if((PLL.theta_est < SLIGHTLY_LESS_THAN_2_PI)&&(PLL.theta_est > THETA_MIN_GARANTEED_CLOSE)){/*If we are some time after the 0-crossing, we are sure the relay has closed and we can turn off the precharge*/
				inverter_safety.contactor_state = CONTACTOR_AC_CHARGE; // go to the CHARGE state;

				// CHARGE state entry code here
				charge_timer = CHARGE_TIMEOUT;// Set the precharge timer
				// CHARGE state entry code end

			}
			break;

		case CONTACTOR_AC_CHARGE:

			//**** Turn on only relay here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON
			RED_LED_PIN = FALSE;// Turn on red LED

			/* Timeout here*/
			if(charge_timer){ /* If time left on the timer*/
				charge_timer--; /* Decrease timer*/
			}else{ /* Else timer has elapsed */

				// CHARGE state exit code would go here

				inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
				inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling
				inverter_faults.charge_timeout_fault = TRUE; // To save the fault event
				// OFF state entry code would go here
			}

			if((!inverter_safety.UV_V_DC_minus) && (!inverter_safety.UV_V_DC_plus)){/* If no undervoltage condition is present*/

				inverter_safety.contactor_state = OPER_AC_ON; // go to the AC_ON state;

				// CHARGE state exit code here
				precharge_timer = 0;
				// CHARGE state exit code end

				// OPER_AC_ON state entry code here
				if(g_New_startup_from_user){
					g_New_startup_from_user = false;
					inverter_faults.reset = true;
				}
				// OPER_AC_ON state entry code end

			};
			break;

		case OPER_AC_ON:

			//**** Turn on only relay here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON
			RED_LED_PIN = FALSE;// Turn on red LED

			// No exit condition other than user OFF or PLL desync;
			break;

		default: // If unknown state
			inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
			//Set all relays off here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = TRUE; // Relay
			RED_LED_PIN = TRUE;// Turn OFF red LED

	}
/**************End of state machine for startup and operational conditions**************************************************/

}

void inverter_medium_freq_task(void){
	/* Medium frequency -> Use T_SYSTICK */

	/* Acesses to inverter variables may be shifted by a few samples in this function because this task has lower priority */
	/*and is preempted by the high frequency task (inverter_control_main) */

	if(inverter_faults.reset){
		inverter_reset_main_errors();
	}

	/* Calculate the (DC) current setpoint (i) */
	inverter_calc_I_D(); /* Direct axis */
	inverter_calc_I_balance(); /* balancing component */

	autozero_state_machine();/* Autozeroing state machine */


}

void inverter_calc_I_D(void){

	//static float V_DC_total_filtered;

	inverter.V_DC_total_filtered = inverter.V_DC_total_filtered + (((measurements_in.V_DC_total-inverter.V_DC_total_filtered)*T_SYSTICK)/VBUS_TOTAL_LPF_TAU); // Low pass filter

	float err_V_DC_total = inverter.V_DC_total_filtered - inverter_setpoints.V_DC_total_setpoint; // Calculate the error

	err_V_DC_total *= VBUS_TOTAL_KP; /* Multiply by the gain to determine how big of a correction to apply */

	/* Saturate */
	if (err_V_DC_total > I_D_MAX){
		err_V_DC_total = I_D_MAX;
	}else if (err_V_DC_total < -I_D_MAX){
		err_V_DC_total = -I_D_MAX;
	}

	inverter.I_D = err_V_DC_total; /* Return the value of the "direct" current */

}

void inverter_calc_I_balance(void){


	//static float V_DC_diff_filtered;

	inverter.V_DC_diff_filtered = inverter.V_DC_diff_filtered + (((measurements_in.V_DC_diff-inverter.V_DC_diff_filtered)*T_SYSTICK)/VBUS_DIFF_LPF_TAU); // Low pass filter

	float err_V_DC_diff = inverter.V_DC_diff_filtered - inverter_setpoints.V_DC_diff_setpoint; // Calculate the error

	err_V_DC_diff *= VBUS_DIFF_KP; /* Multiply by the gain to determine how big of a correction to apply */

	/* Saturate */
	if (err_V_DC_diff > I_BALANCE_MAX){
		err_V_DC_diff = I_BALANCE_MAX;
	}else if (err_V_DC_diff < -I_BALANCE_MAX){
		err_V_DC_diff = -I_BALANCE_MAX;
	}

	inverter.I_balance = err_V_DC_diff; /* Return the value of the "direct" current */

}

void inverter_reset_main_errors(void){

	inverter_faults.reset = false;
	inverter_faults.PLL_sync_fault = false;
	inverter_faults.i_sync_fault = false;
	inverter_faults.OV_v_AC_fault = false;
	inverter_faults.OV_V_DC_plus_fault = false;
	inverter_faults.OV_V_DC_minus_fault = false;
	inverter_faults.UV_V_DC_plus_fault = false;
	inverter_faults.UV_V_DC_minus_fault = false;
	inverter_faults.UV2_V_DC_plus_fault = false;
	inverter_faults.UV2_V_DC_minus_fault = false;
	inverter_faults.OV_V_DC_diff_fault = false;
	//inverter_faults.precharge_timeout_fault; No need to reset these errors here since if they're triggered, the OPER_AC_ON state isn't reached
	//inverter_faults.charge_timeout_fault;

}

void inverter_reset_charge_errors(void){

	inverter_faults.precharge_timeout_fault = false;
	inverter_faults.charge_timeout_fault = false;
}



