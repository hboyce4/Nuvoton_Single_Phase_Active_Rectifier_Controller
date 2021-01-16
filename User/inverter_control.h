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
#include "stdbool.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum {OPEN = 0, CLOSED = 1, PRECHARGE = 2} contactor_state_t;

typedef struct {
	volatile bool OV_v_AC;
	volatile bool OV_V_DC_plus;
	volatile bool OV_V_DC_minus;
	volatile bool UV_V_DC_plus;
	volatile bool UV_V_DC_minus;
	volatile bool OV_V_DC_diff;
	volatile bool HT_Transformer;
	volatile bool OT_Transformer;
	volatile bool HT_Inverter;
	volatile bool OT_Inverter;
	//volatile bool OT_Control_PCB;
	volatile contactor_state_t DC_contactor_state;
	volatile contactor_state_t AC_contactor_state;
	volatile bool PLL_sync;
	volatile bool i_sync;
} inverter_state_safety_t;

typedef struct inverter_state_variables{

	volatile float V_AC_RMS;
	volatile float V_DC_plus;
	volatile float V_DC_minus;
	volatile float V_DC_total;
	volatile float V_DC_diff;
	volatile float I_D;
	//volatile float I_Q;
	volatile float I_AC_RMS;
	volatile float P_AC_RMS;

	volatile float v_AC;
	volatile float i_SP;
	volatile float i_PV;

	volatile float T_transformer;
	volatile float T_inverter;

} inverter_state_variables_t;

float some_float_state_variable;

typedef struct {

	//volatile float I_Q;
	volatile bool inverter_active;
	volatile float some_setpoint;
	volatile float V_DC_total_setpoint;
	volatile float V_DC_diff_setpoint;

} inverter_setpoints_t;

typedef struct {

	//volatile float I_Q;
	volatile float some_limit;

} inverter_limits_t;



/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern inverter_state_variables_t inverter_state_variables;
extern inverter_state_safety_t inverter_state_safety;
extern inverter_setpoints_t inverter_setpoints;
extern inverter_limits_t inverter_limits;


/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/



#endif /* INVERTER_CONTROL_H_ */
