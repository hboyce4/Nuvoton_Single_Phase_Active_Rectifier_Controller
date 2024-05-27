/*
 * PLL.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

 /*  Created on: 2020-11-21
 *      Author: Hugo Boyce
 */

#include "PLL.h"
#include <math.h>
#define ARM_MATH_CM4
#include <arm_math.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

PLL_state_variables_t PLL = {.PI_ctrl_integ_term = 0,
							.w_est = 2*PI_F*NETWORK_FREQ_F,
							.theta_est = 0};

float sin_table[SIN_TABLE_SIZE];

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void init_sin_table(float* sin_table, uint8_t table_size){

	uint8_t i;

	for (i = 0; i <= table_size; i++){

		sin_table[i] = sin((((float)i)/table_size)*(PI/2));


	}

}

void PLL_main(void){ // Service the PLL. Needs up-to-date analog input values.

/*********************Input waveforms calculation begin**********************************/
	float a_alpha = measurements_in.v_AC_n;
	float a_beta = delay_line(measurements_in.v_AC_n);
/*********************Input waveforms calculation end************************************/

	float old_b_beta = PLL.b_beta; //Save the old beta for zero crossing detection

/*********************Estimated waveforms calculation begin******************************/
	float b_alpha = cos_LUT(PLL.theta_est, sin_table);
	PLL.b_beta = sin_LUT(PLL.theta_est, sin_table);
/***********************Estimated waveforms calculation end******************************/

/*************************Zero crossing detection begin*********************************/
	if(signbit(old_b_beta) != signbit(PLL.b_beta)){
		PLL.zero_crossing = true;
	}else{
		PLL.zero_crossing = false;
	}
/***************************Zero crossing detection end**********************************/

/*********************Error calcultion begin*********************************************/
	float err;
	err = (a_alpha*b_alpha) + (a_beta*PLL.b_beta);
/*********************Error calculation end**********************************************/


/****************************PI Controller begin***************************************/
	PLL.PI_ctrl_integ_term += err * PLL_KI * T_CALC; /* integration de l'erreur avec le gain integral */

	if (PLL.PI_ctrl_integ_term > 2*PI_F*MAX_FREQ_DEVIATION){ /* Saturation de la frequence pour empecher le PLL de partir a la derive si il y a perte de verouillage */

		PLL.PI_ctrl_integ_term = 2*PI_F*MAX_FREQ_DEVIATION;

	}else if (PLL.PI_ctrl_integ_term < -2*PI_F*MAX_FREQ_DEVIATION){

		PLL.PI_ctrl_integ_term = -2*PI_F*MAX_FREQ_DEVIATION;

	}

	float old_w_est = PLL.w_est; /* Save the old w_est for Tustin/Bilinear/Trapezoidal integration */

	/* If the inverter mode is one of the open loop modes
	 * (Either AC constant voltage or constant current with OL)*/
	if(inverter.operation_mode == MODE_CONSTANT_AC_CURRENT_OL || inverter.operation_mode == MODE_CONSTANT_AC_VOLTAGE){
		//Keep the Integral term reset
		//PLL.PI_ctrl_integ_term = 0;
		// Force the frequency to the network frequency
		PLL.w_est = 2*PI_F*NETWORK_FREQ_F;

	}else{/* Else, it is one of the normal closed-loop modes*/
		PLL.w_est = err*PLL_KP + PLL.PI_ctrl_integ_term + 2*PI_F*NETWORK_FREQ_F;	/* Output = err*Kp + err*Ki*(1/s) + Feedforward */
	}

/****************************PI Controller end*****************************************/


/**************************Output integrator begin*************************************/
	PLL.theta_est += (PLL.w_est+old_w_est) * T_CALC / 2;	/* Integration of the instantaneous frequency value to get the angle*/
	/* Tustin/Bilinear/Trapezoidal integration method instead of rectangular/Euler method. It's more accurate.*/

	if (PLL.theta_est >= 2*PI_F){	/* Saturation pour garder la valeur de theta_est entre 0 et 2*pi*/

		PLL.theta_est -= 2*PI_F;
	}
/**************************Output integrator end***************************************/

	/* To be put somewhere else */
	PLL.freq_Hz = PLL.w_est*(1/(2*PI_F));

}


float delay_line(float input_value){

	static float delay_array[DELAY_ARRAY_SIZE] = {0};
	static uint8_t read_pointer = 0;

	/* Ici, read_pointer repr�sente l'emplacement qui a �t� lu la derni�re fois*/

	delay_array[read_pointer] = input_value;

	if (read_pointer >= DELAY_ARRAY_SIZE-1){
		read_pointer = 0;
	} else {
		read_pointer++;
	}

	/* Maintenant, read_pointer repr�sente la position � lire maintenant*/

	return delay_array[read_pointer];
}

float sin_LUT(float angle, float* table){


	if((angle >= 0) && (angle < 2*PI_F)){


		if (angle < PI_F/2){

			return table[(uint8_t)((angle/(PI_F/2))*SIN_TABLE_SIZE)]; // casting float to integer causes rounding towards zero. "angle/(PI_F/2)" will always be smaller than 1 here.

		} else if (angle < PI_F){

			return table[(uint8_t)(((PI_F-angle)/(PI_F/2))*SIN_TABLE_SIZE)];

		} else if (angle < (3*PI_F)/2){

			return -table[(uint8_t)(((angle-PI_F)/(PI_F/2))*SIN_TABLE_SIZE)];

		} else {

			return -table[(uint8_t)((((2*PI_F)-angle)/(PI_F/2))*SIN_TABLE_SIZE)];
		}

	} else {

		return 0;
	}

}

float cos_LUT(float angle, float* table){

	if((angle >= 0) && (angle < 2*PI_F)){


		if (angle < PI_F/2){

			return table[(uint8_t)((((PI_F/2)-angle)/(PI_F/2))*SIN_TABLE_SIZE)];

		} else if (angle < PI_F){

			return -table[(uint8_t)(((angle-(PI_F/2))/(PI_F/2))*SIN_TABLE_SIZE)];

		} else if (angle < (3*PI_F)/2){

			return -table[(uint8_t)(((((3*PI_F)/2)-angle)/(PI_F/2))*SIN_TABLE_SIZE)];

		} else {

			return table[(uint8_t)(((angle-((3*PI_F)/2))/(PI_F/2))*SIN_TABLE_SIZE)];
		}

	} else {

		return 1;
	}

}


