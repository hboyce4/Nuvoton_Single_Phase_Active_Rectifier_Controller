/*
 * PLL.h
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#ifndef PLL_H_
#define PLL_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#define NETWORK_FREQ		60							// [Hz] Fréquence nominale du réseau électrique

#define DELAY_ARRAY_SIZE 	(F_CALC/(4*NETWORK_FREQ))	// Taille du tableau pour créer un délai de signal équivalent à pi/4 [rad] à la fréquence de NETWORK_FREQ
#define SIN_TABLE_SIZE		200							// Taille de la LUT de la fct. sinus (max. 254)
#define MAX_FREQ_DEVIATION	6							// [Hz] Déviation maximale à partir de la fréquence nominale permise pour le verouillage
#define PLL_KI				2500						// Gain intégral du compensateur PI
#define	PLL_KP				50							// Gain proportionnel du compensateur PI

// Tolerance for synchronisation
#define PLL_SYNC_TOL 				0.25 /* [units] Tolerance within which the signal is considered in sync */
#define PLL_SYNC_COUNT_FOR_SET		600	/* [counts] Number of times the signal must be found within tolerance to be considered in sync */
#define PLL_SYNC_COUNT_FOR_RESET	10	/* [counts] Number of times the signal must be found OUT of tolerance to be considered out of sync */

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct {

	//volatile float I_Q;
	volatile float PI_ctrl_integ_term;
	volatile float w_est;
	volatile float theta_est;
	volatile float b_beta;
	volatile float freq_Hz; /* Frequency value, low pass filtered and converted to hertz*/
	volatile bool sync;

} PLL_state_variables_t;



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables to be made available externally                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern PLL_state_variables_t PLL;

extern float sin_table[];

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/

void init_sin_table(float*, uint8_t);	// Initialise le tableau de référence pour la fonction sinus (Look-up table, LUT)

void PLL_main(void);			// Fonction principale du PLL. Prend en entrée une onde sinusoidale normalisée à une amplitude d'environ 1
float delay_line(float);		// Retourne le signal donné en entrée avec un délai de DELAY_ARRAY_SIZE échantillons.
float sin_LUT(float, float*); 	// Returns the sine of an angle if it is between 0 and 2*pi. Returns 0 otherwise.
float cos_LUT(float, float*); 	// Returns the cosine of an angle if it is between 0 and 2*pi. Returns 0 otherwise. Takes the same sin_table as sin_LUT for an input

void PLL_check_sync(void);

#endif /* PLL_H_ */
