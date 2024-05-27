/*
 * inverter_control.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#include "inverter_control.h"
#include "sys.h"
#include <math.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Local Macros           				                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

#define DELAY_OFF_FOR_MODE_CHANGE 1000 // milliseconds. Time the contactors need to have been off to allow a mode change.

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

inverter_state_variables_t inverter = {.I_D = 0,
									.operation_mode = MODE_DC_REGULATION};

inverter_state_safety_t inverter_safety;
inverter_state_setpoints_t inverter_setpoints = {.contactor_close_request = 0,
												 .precharge_threshold = UV2_LIMIT,
												 .V_DC_total_setpoint = VBUS_TOTAL_DEFAULT,
												 .V_DC_diff_setpoint = VBUS_DIFF_DEFAULT,
												.V_AC_setpoint = 0,
												.I_AC_setpoint = 0,
												.requested_operation_mode = MODE_DC_REGULATION,
												.requested_sw_en = 0};
inverter_faults_t inverter_faults;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/



void inverter_control_main(void){ // Service the inverter. Needs up-to-date analog input values.
	/* High frequency -> Use T_CALC */

	ENABLE_PULSE_PIN = false;
	//inverter_safety.d_ff_ok = false;

	PLL_main(); // Service the PLL. Needs up-to-date analog input values.

	//Now that the PLL is up-to-date, update all variables and flags
	inverter_check_PLL_sync();
	inverter_check_i_sync();
	inverter_check_voltage_limits();
	inverter_calc_d_ff();

	inverter_contactor_service();

	if(inverter_safety.d_ff_ok){
		ENABLE_PULSE_PIN = true; /* rising edge to enable the inverter*/
	}

	BLUE_LED_PIN = !LATCH_Q_PIN;/* Make the blue LED display the status of the current latch*/

	//Calculate the current setpoint
	inverter.i_SP = (inverter.I_D * PLL.b_beta) + inverter.I_balance ; /* The current setpoint is the sum of the "direct" (iD) current and the balancing current*/
	// inverter.I_D is a continuous ("DC") value, PLL.b_beta is a sine wave.  inverter.I_balance is a proportionately small "DC" value.

}


void inverter_contactor_service(void){


	/* Calculate the contactor state*/
	switch(inverter.operation_mode){

	 case MODE_DC_REGULATION:
		 inverter_contactor_control_DC_regulation_mode();
		 break;

	 case MODE_CONSTANT_AC_CURRENT_PLL:
		 inverter_contactor_control_constant_AC_current_PLL_mode();
		 break;

	 case MODE_CONSTANT_AC_CURRENT_OL:
	 	inverter_contactor_control_constant_AC_current_OL_mode();
	 	break;

	 case MODE_CONSTANT_AC_VOLTAGE:
		 inverter_contactor_control_constant_AC_voltage_mode();
		 break;

	 default:
		 /* Set everything off*/
		 inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
		 inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling

	}

	// From the inverter_safety.contactor_state calculated above, update the pins to the relays and precharge optos
	inverter_contactor_control_update_outputs();


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
	if(fabsf(measurements_in.V_DC_diff) > DIFF_LIMIT){ /* If VBUS+ is over the overvoltage limit*/
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
	static float previous_d; /* Memory of the previous duty cycle. Used for detecting a zero crossing */

	if(measurements_in.V_DC_total != 0){ /* If we're not dividing by zero*/

		if(inverter.operation_mode == MODE_CONSTANT_AC_VOLTAGE){//If we are in controlled AC voltage mode, we force a duty cycle to impose a voltage (current is uncontrolled)
			d = ((inverter_setpoints.V_AC_setpoint*PLL.b_beta)-measurements_in.V_DC_minus)/measurements_in.V_DC_total; // Calc the duty cycle
		}else{// Else the feedforward D term is a slave to the voltage measured.
			d = (measurements_in.v_AC-measurements_in.V_DC_minus)/measurements_in.V_DC_total; // Calc the duty cycle
		}


		/* Saturate between 0 and 1 (0% and 100%) and make the value available for the analog output*/
		if(d < 0){
			inverter.d_feedforward = 0;
		}else if(d > 1){
			inverter.d_feedforward = 1;
		} else {
			inverter.d_feedforward = d;
		}

		/********************** duty cycle OK calculation starts **********************/
		/*
		 * The duty cycle OK flag gets set to OK at a zero crossing
		 *
		 * The duty cycle OK flag gets reset (not OK) if it gets within a certain margin (too close to) of 0 or 1 (0% or 100%).
		 *
		 * */

		if(PLL.zero_crossing){// If a zero crossing is detected
			// If means we crossed zero
			inverter_safety.d_ff_ok = true;
		}

		if((d < D_MARGIN) || (d > 1-D_MARGIN)){ /*If the duty cycle is not within the inverter's capabilities (within the margins) */
			inverter_safety.d_ff_ok = false;
		}
		/********************** duty cycle OK calculation ends **********************/


	}else{
		inverter.d_feedforward = 0.5; /* don't do the calculation and just pick 50% */
		inverter_safety.d_ff_ok = false;
	}

	//Update the memory of the previous value with the one we processed
	previous_d = d;

}

void inverter_contactor_control_DC_regulation_mode(void){

/**************State machine for startup and operational conditions**************************************************/

	static uint32_t precharge_timer = 0;
	static uint32_t charge_timer = 0;


	if((!inverter_safety.PLL_sync)||(!inverter_setpoints.contactor_close_request)){ // If no PLL sync or inverter set to off by user
		inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
	}

	switch(inverter_safety.contactor_state){

		case CONTACTOR_OFF:

			// All contactors and optos are open as a result of this state

			if(inverter_safety.PLL_sync && inverter_setpoints.contactor_close_request){ // If we have PLL sync and the inverter is set to ON

				inverter_safety.contactor_state = CONTACTOR_AC_PRECHARGE; // Go into precharge state

				// PRECHARGE state entry code begins
				precharge_timer = PRECHARGE_TIMEOUT;// Set the precharge timer
				// PRECHARGE state entry code end

				//CONTACTOR_OFF state exit code would go below

			}
			break;


		case CONTACTOR_AC_PRECHARGE:

			// The AC precharge opto is closed as a result of this state. All others are open.

			/* Timeout here*/
			if(precharge_timer){ /* If time left on the timer*/
				precharge_timer--; /* Decrease timer*/
			}else{ /* Else timer has elapsed */

				// PRECHARGE state exit code below
				inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
				inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling
				inverter_faults.precharge_timeout_fault = TRUE; // To save the fault event

				// CONTACTOR_OFF state entry code would go here
			}

			/* Check that the voltages are high enough so the "charge" phase will not draw too large of a current spike
			 * and check that the angle is at the right moment, so the relay contacts close near the zero crossing.
			 * If Bus voltages are high enough and phase angle is good, then*/
			if((!inverter_safety.UV2_V_DC_plus)&&(!inverter_safety.UV2_V_DC_minus)&&(PLL.theta_est > THETA_MIN_RELAY_CLOSE)&&(PLL.theta_est < THETA_MAX_RELAY_CLOSE)){
				/*The moment is right, reset the precharge timer, and command the AC relay to close by going to the WAIT_FOR_CLOSE state*/

				// PRECHARGE state exit code below
				precharge_timer = 0;
				inverter_safety.contactor_state = CONTACTOR_AC_WAIT_FOR_CLOSE; // go to the WAIT_FOR_CLOSE state
				// PRECHARGE state exit code end

				// WAIT_FOR_CLOSE state entry code would go here

			}
			break;


		case CONTACTOR_AC_WAIT_FOR_CLOSE:

			// AC precharge opto AND AC relay are closed as a result of this state. Others are open.

			/* If the end of the cycle and the zero crossing is near*/
			if(PLL.theta_est > SLIGHTLY_LESS_THAN_2_PI){
				// This means we are very near the point where the relay contacts actually close
				inverter_safety.contactor_state = CONTACTOR_AC_DWELL; // go to the dwell state;
			}

			break;

		case CONTACTOR_AC_DWELL:

			// AC precharge opto AND AC relay are still closed in this state. Others are open.
			// No change from CONTACTOR_AC_WAIT_FOR_CLOSE

			//TODO Maybe this state could be merged with CONTACTOR_AC_WAIT_FOR_CLOSE

			if((PLL.theta_est < SLIGHTLY_LESS_THAN_2_PI)&&(PLL.theta_est > THETA_MIN_GARANTEED_CLOSE)){/*If we are some time after the 0-crossing*/
				/* We are sure the relay has closed and we can turn off the precharge*/

				// CONTACTOR_AC_DWELL state exit code would go below

				// AC_CHARGE state entry code here
				inverter_safety.contactor_state = CONTACTOR_AC_CHARGE; // go to the CHARGE state;
				charge_timer = CHARGE_TIMEOUT;// Set the precharge timer
				// AC_CHARGE state entry code end


			}
			break;

		case CONTACTOR_AC_CHARGE:

			/* For this state, only the AC relay is closed */

			/* Timeout here*/
			if(charge_timer){ /* If time left on the timer*/
				charge_timer--; /* Decrease timer*/
			}else{ /* Else timer has elapsed */
				// Charge is taking too long, go to OFF state

				// AC_CHARGE state exit code goes below
				inverter_faults.charge_timeout_fault = TRUE; // To save the fault event

				// CONTACTOR_OFF state entry code below
				inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
				inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling


			}

			if((!inverter_safety.UV_V_DC_minus) && (!inverter_safety.UV_V_DC_plus)){/* If no undervoltage condition is present*/


				// AC_CHARGE state exit code below
				precharge_timer = 0;

				//AC_CLOSED_DC_OPEN state entry code below
				inverter_safety.contactor_state = CONTACTOR_AC_CLOSED_DC_OPEN; // go to the AC_ON state;

				if(g_New_startup_from_user){
					g_New_startup_from_user = false;
					inverter_faults.reset = true;
				}


			};
			break;

		case CONTACTOR_AC_CLOSED_DC_OPEN:

			/* For this state, only the AC relay is closed */

			// No exit condition other than user OFF or PLL desync;

			break;

		default: // If unknown state

			//Go to CONTACTOR_OFF state
			inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
			inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling

			// All relays and optos will be opened as a result of this state


	}
/**************End of state machine for startup and operational conditions**************************************************/

}

void inverter_contactor_control_constant_AC_current_PLL_mode(void){
 /* Do nothing for now */
}

void inverter_contactor_control_constant_AC_current_OL_mode(void){
 /* Do nothing for now */
}

void inverter_contactor_control_constant_AC_voltage_mode(void){
	switch(inverter_safety.contactor_state){

			case CONTACTOR_OFF:

				break;


			case CONTACTOR_AC_PRECHARGE:

				break;

			case CONTACTOR_AC_WAIT_FOR_CLOSE:

				break;

			case CONTACTOR_AC_DWELL:

				break;

			case CONTACTOR_AC_CHARGE:

				break;

			case CONTACTOR_AC_CLOSED_DC_OPEN:

				break;

			default: // If unknown state
				inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
				inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling

		}
}

void inverter_contactor_control_update_outputs(void){

	switch(inverter_safety.contactor_state){

		case CONTACTOR_OFF:

			//Set all relays off here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = TRUE; // Relay
			DC_PRECHARGE_PIN = TRUE; // Precharge opto
			DC_RELAY_PIN = TRUE; // Relay

			RED_LED_PIN = TRUE;// Turn OFF red LED
			break;


		case CONTACTOR_AC_PRECHARGE:

			// Turn on only precharge opto here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // AC Precharge opto ON
			AC_RELAY_PIN = TRUE; // AC Relay OFF
			DC_PRECHARGE_PIN = TRUE; // DC Precharge opto OFF
			DC_RELAY_PIN = TRUE; // DC Relay OFF

			RED_LED_PIN = TRUE;// Turn OFF red LED

			break;


		case CONTACTOR_AC_WAIT_FOR_CLOSE:

			//**** Turn on precharge opto AND AC relay here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON

			//DC relays disconnected
			DC_PRECHARGE_PIN = TRUE; // Precharge opto
			DC_RELAY_PIN = TRUE; // Relay

			RED_LED_PIN = FALSE;// Turn on red LED

			break;

		case CONTACTOR_AC_DWELL:

			//**** Turn on precharge opto AND AC relay here (signals are active low)
			AC_PRECHARGE_PIN = FALSE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON
			DC_PRECHARGE_PIN = TRUE; // DC Precharge opto OFF
			DC_RELAY_PIN = TRUE; // DC Relay OFF
			RED_LED_PIN = FALSE;// Turn on red LED

			break;

		case CONTACTOR_AC_CHARGE:

			//**** Turn on only AC relay here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON

			DC_PRECHARGE_PIN = TRUE; // DC Precharge opto OFF
			DC_RELAY_PIN = TRUE; // DC Relay OFF

			RED_LED_PIN = FALSE;// Turn on red LED

			break;

		case CONTACTOR_AC_CLOSED_DC_OPEN:

			//**** Turn on only relay here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = FALSE; // Relay ON

			DC_PRECHARGE_PIN = TRUE; // DC Precharge opto OFF
			DC_RELAY_PIN = TRUE; // DC Relay OFF

			RED_LED_PIN = FALSE;// Turn on red LED

			break;

		default: // If unknown state
			inverter_safety.contactor_state = CONTACTOR_OFF; // switch to the OFF state immediately
			inverter_setpoints.contactor_close_request = FALSE; // To avoid cycling
			//Set all relays off here (signals are active low)
			AC_PRECHARGE_PIN = TRUE; // Precharge opto
			AC_RELAY_PIN = TRUE; // Relay
			DC_PRECHARGE_PIN = TRUE; // DC Precharge opto OFF
			DC_RELAY_PIN = TRUE; // DC Relay OFF

			RED_LED_PIN = TRUE;// Turn OFF red LED

	}



}


void inverter_medium_freq_task(void){
	/* Medium frequency -> Use T_SYSTICK */

	/* Accesses to inverter variables may be shifted by a few samples in this function because this task has lower priority */
	/*and is preempted by the high frequency task (inverter_control_main) */

	if(inverter_faults.reset){
		inverter_reset_main_errors();
	}

	/* Calculate the (DC) current setpoint (i) */
	//TODO: Move this to the high frequency task
	inverter_calc_I_D(); /* Direct axis */
	inverter_calc_I_balance(); /* balancing component */

	autozero_state_machine();/* Autozeroing state machine */

	inverter_mode_change(); /* The mode change request service routine*/

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


void inverter_mode_change(void){

	static uint32_t delay_off_for_mode_change_cnt;

	// IF all the contactors are off
	if(inverter_safety.contactor_state == CONTACTOR_OFF){/* The mode can ONLY be changed if all contactors are off */

		// If the counter has reched its end value
		if(delay_off_for_mode_change_cnt >= DELAY_OFF_FOR_MODE_CHANGE){
			//We can now proceed with the change
			inverter.operation_mode = inverter_setpoints.requested_operation_mode;

		}else{// Else the counter has not yet reached the end
			// Increment the counter
			delay_off_for_mode_change_cnt++;
		}

	}else{ // Else the contactors are not off
		//reset the counter
		delay_off_for_mode_change_cnt = 0;
	}

}


void inverter_req_next_mode(void){

	inverter_setpoints.requested_operation_mode++; /* Increment the mode */
	if(inverter_setpoints.requested_operation_mode >= MODE_LAST){ /*If the last mode is reached*/
		inverter_setpoints.requested_operation_mode = 0; /* Go to the first mode */
	}

}


void inverter_req_prev_mode(void){

	if(inverter_setpoints.requested_operation_mode <= 0){/* If we're at mode zero, there is no previous mode so go back to last mode*/
		inverter_setpoints.requested_operation_mode = MODE_LAST - 1;
	}else{
		inverter_setpoints.requested_operation_mode--; /* Decrement the mode */
	}

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



