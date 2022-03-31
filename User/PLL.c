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

PLL_state_variables_t PLL = {.PI_ctrl_integ_term = 0,
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

void PLL_main(void){ // Service the PLL. Needs up-to-date analog input values.

/*********************Input waveforms calculation begin**********************************/
	float a_alpha = inverter.v_AC_n;
	float a_beta = delay_line(inverter.v_AC_n);
/*********************Input waveforms calculation end************************************/


/*********************Estimated waveforms calculation begin******************************/
	float b_alpha = cos_LUT(PLL.theta_est, sin_table);
	PLL.b_beta = sin_LUT(PLL.theta_est, sin_table);
/*********************Estimated waveforms calculation end******************************/


/*********************Error calcultion begin*********************************************/
	float err;
	err = (a_alpha*b_alpha) + (a_beta*PLL.b_beta);
/*********************Error calculation end**********************************************/


/****************************PI Controller begin***************************************/

	PLL.PI_ctrl_integ_term += err * PLL_KI * T_CALC; /* intégration de l'erreur avec le gain intégral */

	if (PLL.PI_ctrl_integ_term > 2*M_PI*MAX_FREQ_DEVIATION){ /* Saturation de la fréquence pour empêcher le PLL de partir à la dérive si il y a perte de verouillage*/

		PLL.PI_ctrl_integ_term = 2*M_PI*MAX_FREQ_DEVIATION;

	}else if (PLL.PI_ctrl_integ_term < -2*M_PI*MAX_FREQ_DEVIATION){

		PLL.PI_ctrl_integ_term = -2*M_PI*MAX_FREQ_DEVIATION;

	}

	PLL.w_est = err*PLL_KP + PLL.PI_ctrl_integ_term + 2*M_PI*NETWORK_FREQ;	/* Output = err*Kp + err*Ki*(1/s) + Feedforward */

/****************************PI Controller end*****************************************/


/**************************Output integrator begin*************************************/


	PLL.theta_est += PLL.w_est * T_CALC;	/* Integration of the instantaneous frequency value to get the angle*/

	if (PLL.theta_est >= 2*M_PI){	/* Saturation pour garder la valeur de theta_est entre 0 et 2*pi*/

		PLL.theta_est -= 2*M_PI;
	}

/**************************Output integrator end***************************************/

	/* To be put somewhere else */
	PLL.freq_Hz = PLL.w_est*(1/(2*M_PI));

	PLL_check_sync();
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

void PLL_check_sync(void){

	static uint16_t set_count, reset_count;

	if((inverter.v_AC_n > PLL.b_beta-PLL_SYNC_TOL) && (inverter.v_AC_n < PLL.b_beta+PLL_SYNC_TOL)){ /* If the input voltage is within tolerance of the estimated voltage*/
		reset_count = 0;
		set_count++;
	}else{/* Else it's out of tolerance*/
		reset_count++;
		set_count = 0;
	}


	if(reset_count >= PLL_SYNC_COUNT_FOR_RESET){
		reset_count = PLL_SYNC_COUNT_FOR_RESET;
		PLL.sync = false;
	}else if(set_count >= PLL_SYNC_COUNT_FOR_SET){
		set_count = PLL_SYNC_COUNT_FOR_SET;
		PLL.sync = true;
	}


}

