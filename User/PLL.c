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

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

PLL_state_variables_t PLL_state_variables = {.PI_ctrl_integ_term = 0,
											 .w_est = NETWORK_FREQ,
											 .theta_est = 0};
float sin_table[SIN_TABLE_SIZE];

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

void init_sin_table(float* sin_table, uint8_t table_size){

	uint8_t i;

	for (i = 0; i <= table_size; i++){

		sin_table[i] = sin((((float)i)/table_size)*(M_PI/2));


	}

}

void PLL_main(void){





/*********************Input waveforms calculation begin**********************************/
	float a_alpha = inverter_state_variables.v_AC_n;
	float a_beta = delay_line(inverter_state_variables.v_AC_n);
/*********************Input waveforms calculation end************************************/


/*********************Estimated waveforms calculation begin******************************/
	float b_alpha = cos_LUT(PLL_state_variables.theta_est, sin_table);
	float b_beta = sin_LUT(PLL_state_variables.theta_est, sin_table);
/*********************Estimated waveforms calculation end******************************/


/*********************Error calcultion begin*********************************************/
	float err;
	err = (a_alpha*b_alpha) + (a_beta*b_beta);
/*********************Error calculation end**********************************************/



/****************************PI Controller begin***************************************/

	PLL_state_variables.PI_ctrl_integ_term += err * KI * T_CALC; /* intégration de l'erreur avec le gain intégral */

	if (PLL_state_variables.PI_ctrl_integ_term > 2*M_PI*MAX_FREQ_DEVIATION){ /* Saturation de la fréquence pour empêcher le PLL de partir à la dérive si il y a perte de verouillage*/

		PLL_state_variables.PI_ctrl_integ_term = 2*M_PI*MAX_FREQ_DEVIATION;

	}else if (PLL_state_variables.PI_ctrl_integ_term < -2*M_PI*MAX_FREQ_DEVIATION){

		PLL_state_variables.PI_ctrl_integ_term = -2*M_PI*MAX_FREQ_DEVIATION;

	}

	PLL_state_variables.w_est = err*KP + PLL_state_variables.PI_ctrl_integ_term + 2*M_PI*NETWORK_FREQ;	/* Output = err*Kp + err*Ki*(1/s) + Feedforward */

/****************************PI Controller end*****************************************/



/**************************Output integrator begin*************************************/

	//PLL_state_variables.theta_est = output_integrator(PLL_state_variables.theta_est, PLL_state_variables.w_est, 2*M_PI, T_CALC); // Fonction output_integrator en assembleur


	PLL_state_variables.theta_est += PLL_state_variables.w_est * T_CALC;	/* Integration of the instantaneous frequency value to get the angle*/

	if (PLL_state_variables.theta_est >= 2*M_PI){	/* Saturation pour garder la valeur de theta_est entre 0 et 2*pi*/

		PLL_state_variables.theta_est -= 2*M_PI;
	}

/**************************Output integrator end***************************************/

	//return a_beta; /* Return whatever state variable we are interested in monitoring */

	inverter_state_variables.i_SP = 33*b_beta;//Test

	PLL_state_variables.freq_Hz = PLL_state_variables.w_est*(1/(2*M_PI));
}


float delay_line(float input_value){

	static float delay_array[DELAY_ARRAY_SIZE] = {0};
	static uint8_t read_pointer = 0;

	/* Ici, read_pointer représente l'emplacement qui a été lu la dernière fois*/

	delay_array[read_pointer] = input_value;

	if (read_pointer >= DELAY_ARRAY_SIZE-1){
		read_pointer = 0;
	} else {
		read_pointer++;
	}

	/* Maintenant, read_pointer représente la position à lire maintenant*/

	return delay_array[read_pointer];
}

float sin_LUT(float angle, float* table){


	if((angle >= 0) && (angle < 2*M_PI)){


		if (angle < M_PI/2){

			return table[(uint8_t)((angle/(M_PI/2))*SIN_TABLE_SIZE)]; // casting float to integer causes rounding towards zero. "angle/(M_PI/2)" will always be smaller than 1 here.

		} else if (angle < M_PI){

			return table[(uint8_t)(((M_PI-angle)/(M_PI/2))*SIN_TABLE_SIZE)];

		} else if (angle < (3*M_PI)/2){

			return -table[(uint8_t)(((angle-M_PI)/(M_PI/2))*SIN_TABLE_SIZE)];

		} else {

			return -table[(uint8_t)((((2*M_PI)-angle)/(M_PI/2))*SIN_TABLE_SIZE)];
		}

	} else {

		return 0;
	}

}

float cos_LUT(float angle, float* table){

	if((angle >= 0) && (angle < 2*M_PI)){


		if (angle < M_PI/2){

			return table[(uint8_t)((((M_PI/2)-angle)/(M_PI/2))*SIN_TABLE_SIZE)];

		} else if (angle < M_PI){

			return -table[(uint8_t)(((angle-(M_PI/2))/(M_PI/2))*SIN_TABLE_SIZE)];

		} else if (angle < (3*M_PI)/2){

			return -table[(uint8_t)(((((3*M_PI)/2)-angle)/(M_PI/2))*SIN_TABLE_SIZE)];

		} else {

			return table[(uint8_t)(((angle-((3*M_PI)/2))/(M_PI/2))*SIN_TABLE_SIZE)];
		}

	} else {

		return 1;
	}

}
