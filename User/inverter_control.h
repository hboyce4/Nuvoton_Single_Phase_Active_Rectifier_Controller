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
#include "user_sys.h"
#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#define PWM_CARRIER_FREQ 400000

#define VBUS_TOTAL_DEFAULT 48 /* [V] Default total bus voltage setpoint*/
#define VBUS_TOTAL_LPF_TAU 0.007958 /* [1/s] 20 Hz (1/(2*pi*20)) */
#define VBUS_TOTAL_KP 5 /* Proportional gain of the V_DC_total controller */


#define VBUS_DIFF_DEFAULT 0 /* [V] Default voltage difference setpoint between the two half-busses*/
#define VBUS_DIFF_LPF_TAU 0.026526 /* [1/s] 6 Hz (1/(2*pi*6)) */
#define VBUS_DIFF_KP 1 /* Proportional gain of the V_DC_diff controller */

#define I_D_MAX 33 /* [A] Maximum allowable peak AC current */
#define I_BALANCE_MAX 3 /* [A] Maximum allowable balancing (DC) current */

#define I_SYNC_TOL 				5 	/* [A] Tolerance within which the signal is considered in sync */
#define I_SYNC_COUNT_FOR_SET	200	/* [counts] Number of times the signal must be found within tolerance to be considered in sync */
#define I_SYNC_COUNT_FOR_RESET	5	/* [counts] Number of times the signal must be found OUT of tolerance to be considered out of sync */

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum {OPEN = 0, CLOSED = 1, PRECHARGE = 2} contactor_state_t;

typedef struct {

	/* Fast refresh */
	volatile bool OV_v_AC;
	volatile bool OV_V_DC_plus;
	volatile bool OV_V_DC_minus;
	volatile bool UV_V_DC_plus;
	volatile bool UV_V_DC_minus;
	volatile bool OV_V_DC_diff;
	volatile contactor_state_t DC_contactor_state;
	volatile contactor_state_t AC_contactor_state;
	volatile bool i_sync;

	/* Slow refresh*/
	volatile bool HT_Transformer;
	volatile bool OT_Transformer;
	volatile bool HT_Inverter;
	volatile bool OT_Inverter;

} inverter_state_safety_t;

typedef struct inverter_state_variables{

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

//float some_float_state_variable;

typedef struct {

	//volatile float I_Q;
	volatile bool inverter_active;
	volatile float some_setpoint;
	volatile float V_DC_total_setpoint;
	volatile float V_DC_diff_setpoint;

} inverter_state_setpoints_t;

/*
typedef struct {

	//volatile float I_Q;
	volatile float some_limit;

} inverter_limits_t;
*/


/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern inverter_state_variables_t inverter;
extern inverter_state_safety_t inverter_safety;
extern inverter_state_setpoints_t inverter_setpoints;
//extern inverter_limits_t inverter_limits;


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/

void init_inverter_control(void);
void inverter_control_main(void);
void inverter_safety_fast(void);
void inverter_calc_I_D(void);
void inverter_calc_I_balance(void);
void inverter_check_i_sync(void);

#endif /* INVERTER_CONTROL_H_ */
