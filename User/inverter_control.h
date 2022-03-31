/*
  * inverter_control.h
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#ifndef INVERTER_CONTROL_H_
#define INVERTER_CONTROL_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "stdbool.h"
#include "NuMicro.h"
#include "main.h"
#include "constants.h"
#include "analog.h"
#include "UART_over_DMA.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Local Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/


// VBUS_TOTAL Filter
#define VBUS_TOTAL_LPF_TAU 0.007958 /* [1/s] 20 Hz (1/(2*pi*20)) */
#define VBUS_TOTAL_KP 5 /* Proportional gain of the V_DC_total controller */

// VBUS_DIFF Filter
#define VBUS_DIFF_LPF_TAU 0.026526 /* [1/s] 6 Hz (1/(2*pi*6)) */
#define VBUS_DIFF_KP 1 /* Proportional gain of the V_DC_diff controller */

// Maximum currents
#define I_D_MAX 33 /* [A] Maximum allowable peak AC current */
#define I_BALANCE_MAX 3 /* [A] Maximum allowable balancing (DC) current */


// Tolerances for synchronization
#define I_SYNC_TOL 				5 	/* [A] Tolerance within which the signal is considered in sync */
#define I_SYNC_COUNT_FOR_SET	200	/* [counts] Number of times the signal must be found within tolerance to be considered in sync */
#define I_SYNC_COUNT_FOR_RESET	5	/* [counts] Number of times the signal must be found OUT of tolerance to be considered out of sync */

// Operational limits
#define OV_LIMIT 30 /* [V] Voltage limit for overvoltage on either VBUS */
#define UV_LIMIT 15 /* [V] Undervoltage limit 1 on either VBUS. Below this, inverter must stop */
#define UV2_LIMIT 8 /* [V] Undervoltage limit 2 on either VBUS. Below this, precharge is necessary */
#define DIFF_LIMIT 3 /* [V] Maximum voltage imbalance between VBUSes */

// Limits for startup on AC
// The AC relay takes 10ms to close. Therefore it should be turned on a bit more than 10ms before the next zero-crossing
// We define a window of phase angle where it is appropriate to close the AC relay
// The ideal trigger point is defined as ((1/60Hz)-10ms / (1/60Hz))*2*pi =  2.51327 rad
#define THETA_MIN_RELAY_CLOSE 2.1363 //[rad] Is equal to ((1/60Hz) - 11ms / (1/60Hz)) * 2 * pi
#define THETA_MAX_RELAY_CLOSE 2.5133 //[rad] Is equal to ((1/60Hz) - 10ms / (1/60Hz)) * 2 * pi
#define SLIGHTLY_LESS_THAN_2_PI 6 // Slightly less than 2*pi=6.28. Determines when the end of a cycle has been reached, and the approximate theta of a zero crossing
#define THETA_MIN_GARANTEED_CLOSE 0.5 // [rad] theta in the cycle following the activation of the relay where we're sure the relay has finished closing
#define PRECHARGE_TIMEOUT 60000 // [T_CALC] Number of time steps after which the PRECHARGE step times out. steps = seconds*F_CALC
#define CHARGE_TIMEOUT 400 // [T_CALC] Number of time steps after which the CHARGE step times out. steps = seconds*F_CALC

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/
//typedef enum {OPEN = 0, CLOSED = 1, PRECHARGE = 2} contactor_state_t;

typedef enum {OFF = 0, PRECHARGE = 1, WAIT_FOR_CLOSE = 2, DWELL = 3, CHARGE = 4, AC_ON = 5} operating_state_t;

typedef struct { /* Safety and operational statuses and conditions */

	/* Fast refresh */
	volatile bool i_sync;
	volatile bool OV_v_AC;
	volatile bool OV_V_DC_plus;
	volatile bool OV_V_DC_minus;
	volatile bool UV_V_DC_plus;
	volatile bool UV_V_DC_minus;
	volatile bool UV2_V_DC_plus;
	volatile bool UV2_V_DC_minus;
	volatile bool OV_V_DC_diff;
	//volatile contactor_state_t DC_contactor_state;
	//volatile contactor_state_t AC_contactor_state;
	volatile operating_state_t operating_state;

	/* Slow refresh*/
	volatile bool HT_Transformer;
	volatile bool OT_Transformer;
	volatile bool HT_Inverter;
	volatile bool OT_Inverter;

} inverter_state_safety_t;

typedef struct inverter_state_variables{ /* Process values or state variables */

	/* "Constant" (slow varying) values for control */
	volatile float V_DC_plus;
	volatile float V_DC_minus;
	volatile float V_DC_total;
	volatile float V_DC_diff;
	volatile float I_D;
	//volatile float I_Q;
	volatile float I_balance;
	volatile float T_transformer;
	volatile float T_inverter;

	/* "Constant" (slow varying) values for power calculation */
	volatile float V_AC_RMS;
	volatile float I_AC_RMS;
	volatile float P_AC_RMS;

	/* Dynamic (fast varying) values for control */
	volatile float v_AC;
	volatile float v_AC_n; /* v_AC normalized to a peak amplitude of one */
	volatile float i_SP;
	volatile float i_PV;
	volatile float d_feedforward;

} inverter_state_variables_t;

float some_float_state_variable; // caca

typedef struct { /* Uset selectable setpoints and modes */

	//volatile float I_Q;
	volatile bool inverter_active;
	volatile float some_setpoint;
	volatile float V_DC_total_setpoint;
	volatile float V_DC_diff_setpoint;

} inverter_state_setpoints_t;


typedef struct { /* Memorizes fault conditions, limits exceeded, etc */

	volatile bool i_sync_fault;
	volatile bool OV_v_AC_fault;
	volatile bool OV_V_DC_plus_fault;
	volatile bool OV_V_DC_minus_fault;
	volatile bool UV_V_DC_plus_fault;
	volatile bool UV_V_DC_minus_fault;
	volatile bool UV2_V_DC_plus_fault;
	volatile bool UV2_V_DC_minus_fault;
	volatile bool OV_V_DC_diff_fault;

} inverter_errors_t;



/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern inverter_state_variables_t inverter; /* Process values or state variables */
extern inverter_state_safety_t inverter_safety; /* Safety and operational statuses */
extern inverter_state_setpoints_t inverter_setpoints; /* Uset selectable setpoints and modes */
extern inverter_errors_t inverter_faults; /* Memorizes fault conditions, broken limits, etc */


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/


void inverter_control_main(void);

void inverter_calc_I_D(void);
void inverter_calc_I_balance(void);
void inverter_check_safety_operational_status(void);
void inverter_check_i_sync(void);
void inverter_check_limits(void);
void inverter_calc_state(void);


#endif /* INVERTER_CONTROL_H_ */
