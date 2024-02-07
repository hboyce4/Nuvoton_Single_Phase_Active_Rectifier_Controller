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
#include <math.h>
#include "stdbool.h"
#include "NuMicro.h"
#include "main.h"
#include "constants.h"
#include "analog.h"
#include "UART_over_DMA.h"
#include "UI.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Local Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

// Tolerance for PLL synchronisation
#define PLL_SYNC_TOL 				0.25 /* [units] Tolerance within which the signal is considered in sync */
#define PLL_SYNC_COUNT_FOR_SET		600	/* [counts] Number of times the signal must be found within tolerance to be considered in sync */
#define PLL_SYNC_COUNT_FOR_RESET	10	/* [counts] Number of times the signal must be found OUT of tolerance to be considered out of sync */


// Tolerances for synchronization
#define I_SYNC_TOL 				5 	/* [A] Tolerance within which the signal is considered in sync */
#define I_SYNC_COUNT_FOR_SET	200	/* [counts] Number of times the signal must be found within tolerance to be considered in sync */
#define I_SYNC_COUNT_FOR_RESET	5	/* [counts] Number of times the signal must be found OUT of tolerance to be considered out of sync */

// Operational limits
#define OV_LIMIT 30.5f /* [V] Voltage limit for overvoltage on either VBUS */
#define UV_LIMIT 14.8f /* [V] Undervoltage limit 1 on either VBUS. Below this, inverter must stop */
#define UV2_LIMIT 9.5f /* [V] Undervoltage limit 2 on either VBUS. Below this, precharge is necessary */
#define DIFF_LIMIT 3.0f /* [V] Maximum voltage imbalance between VBUSes */

// Limits for startup on AC
// The AC relay takes 8ms to close. Therefore it should be turned on approx. 8ms before the next zero-crossing.
// We define a window of phase angle where it is appropriate to close the AC relay
// The ideal trigger point is defined as ((1/60Hz)-8ms / (1/60Hz))*2*pi =  3.2673 rad
#define THETA_MIN_RELAY_CLOSE 3.1730f //[rad] Is equal to ((1/60Hz) - 8.25ms / (1/60Hz)) * 2 * pi
#define THETA_MAX_RELAY_CLOSE 3.3615f //[rad] Is equal to ((1/60Hz) - 7.75ms / (1/60Hz)) * 2 * pi
#define SLIGHTLY_LESS_THAN_2_PI 6.0f // Slightly less than 2*pi=6.28. Determines when the end of a cycle has been reached, and the approximate theta of a zero crossing
#define THETA_MIN_GARANTEED_CLOSE 0.5f // [rad] theta in the cycle following the activation of the relay where we're sure the relay has finished closing
#define PRECHARGE_TIMEOUT 180000 // [T_CALC] Number of time steps after which the PRECHARGE step times out. steps = seconds*F_CALC
#define CHARGE_TIMEOUT 600 // [T_CALC] Number of time steps after which the CHARGE step times out. steps = seconds*F_CALC

// VBUS_TOTAL Filter
#define VBUS_TOTAL_LPF_TAU 0.007958f /* [1/s] 20 Hz (1/(2*pi*20)) */
// VBUS_DIFF Filter
#define VBUS_DIFF_LPF_TAU 0.026526f /* [1/s] 6 Hz (1/(2*pi*6)) */


//#define VBUS_DIFF_KP 1 /* Proportional gain of the V_DC_diff controller */
#define VBUS_DIFF_KP 0.33f /* Proportional gain of the V_DC_diff controller */
//#define VBUS_TOTAL_KP 5 /* Proportional gain of the V_DC_total controller */
#define VBUS_TOTAL_KP (5/3) /* Proportional gain of the V_DC_total controller */


// Maximum currents
//#define I_D_MAX 33 /* [A] Maximum allowable peak AC current */
#define I_D_MAX 11 /* [A] Maximum allowable peak AC current */
//#define I_BALANCE_MAX 3 /* [A] Maximum allowable balancing (DC) current */
#define I_BALANCE_MAX 1 /* [A] Maximum allowable balancing (DC) current */

#define D_MARGIN 0.15 /* Duty cycle margin. If the theoretical duty cycle is smaller than D_MARGIN, or greater than 1-D_MARGIN, don't run the inverter*/

#define AC_PRECHARGE_PIN PC2
#define DC_PRECHARGE_PIN PC3
#define AC_RELAY_PIN PC4
#define DC_RELAY_PIN PC5

#define COMP_RESET_PIN PA5

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/

typedef enum {CONTACTOR_OFF = 0, CONTACTOR_AC_PRECHARGE = 1, CONTACTOR_AC_WAIT_FOR_CLOSE = 2, CONTACTOR_AC_DWELL = 3, CONTACTOR_AC_CHARGE = 4, CONTACTOR_AC_CLOSED_DC_OPEN = 5, CONTACTOR_AC_CLOSED_DC_PRECHARGE = 6, CONTACTOR_AC_DC_CLOSED = 7} contactor_state_t;

typedef enum {MODE_DC_REGULATION = 0, MODE_CONSTANT_AC_CURRENT = 1, MODE_CONSTANT_AC_VOLTAGE = 2, MODE_LAST = 3} operation_mode_t;


typedef struct { /* Safety and operational statuses and conditions */

	/* Fast refresh */
	volatile bool PLL_sync;
	volatile bool i_sync;
	volatile bool d_ff_ok;
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
	volatile contactor_state_t contactor_state;

	/* Slow refresh*/
	volatile bool HT_Transformer;
	volatile bool OT_Transformer;
	volatile bool HT_Inverter;
	volatile bool OT_Inverter;

} inverter_state_safety_t;

typedef struct inverter_state_variables{ /* Process values or state variables */

	/* "Constant" (slow varying) values for control */
	//volatile float V_DC_plus;
	//volatile float V_DC_minus;
	//volatile float V_DC_total;
	volatile float V_DC_total_filtered;
	//volatile float V_DC_diff;
	volatile float V_DC_diff_filtered;
	volatile float I_D;
	//volatile float I_Q;
	volatile float I_balance;
	//volatile float T_transformer;
	//volatile float T_inverter;

	/* "Constant" (slow varying) values for power calculation */
	volatile float V_AC_RMS;
	volatile float I_AC_RMS;
	volatile float P_AC_RMS;

	/* Dynamic (fast varying) values for control */
	//volatile float v_AC;
	//volatile float v_AC_n; /* v_AC normalized to a peak amplitude of one */
	volatile float i_SP;
	//volatile float i_PV;
	volatile float d;
	volatile float d_feedforward;

	volatile operation_mode_t operation_mode;

} inverter_state_variables_t;


typedef struct { /* Uset selectable setpoints and modes */

	//volatile float I_Q;
	volatile bool contactor_close_request;
	//volatile bool latch_set;
	//volatile float some_setpoint;
	volatile float V_DC_total_setpoint;
	volatile float V_DC_diff_setpoint;
	volatile float precharge_threshold;

} inverter_state_setpoints_t;


typedef struct { /* Memorizes fault conditions, limits exceeded, etc */

	volatile bool reset;
	volatile bool PLL_sync_fault;
	volatile bool i_sync_fault;
	volatile bool OV_v_AC_fault;
	volatile bool OV_V_DC_plus_fault;
	volatile bool OV_V_DC_minus_fault;
	volatile bool UV_V_DC_plus_fault;
	volatile bool UV_V_DC_minus_fault;
	volatile bool UV2_V_DC_plus_fault;
	volatile bool UV2_V_DC_minus_fault;
	volatile bool OV_V_DC_diff_fault;
	volatile bool precharge_timeout_fault;
	volatile bool charge_timeout_fault;

} inverter_faults_t;



/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern inverter_state_variables_t inverter; /* Process values or state variables */
extern inverter_state_safety_t inverter_safety; /* Safety and operational statuses */
extern inverter_state_setpoints_t inverter_setpoints; /* User selectable setpoints and modes */
extern inverter_faults_t inverter_faults; /* Memorizes fault conditions, broken limits, etc */


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/


void inverter_control_main(void);

void inverter_check_PLL_sync(void);
void inverter_check_i_sync(void);

void inverter_check_safety_operational_status(void);
void inverter_check_voltage_limits(void);

void inverter_calc_d_ff(void);// Calculate duty cycle feedforward term


void inverter_contactor_control_DC_regulation_mode(void);
void inverter_contactor_control_constant_AC_current_mode(void);
void inverter_contactor_control_constant_AC_voltage_mode(void);

void inverter_medium_freq_task(void);

void inverter_calc_I_D(void);
void inverter_calc_I_balance(void);

void inverter_try_next_mode(void);
void inverter_try_prev_mode(void);

void inverter_reset_main_errors(void);
void inverter_reset_charge_errors(void);

#endif /* INVERTER_CONTROL_H_ */
