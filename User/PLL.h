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
#include <stdbool.h>
#include "main.h"
#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#define NETWORK_FREQ		60							// [Hz] Fr�quence nominale du r�seau �lectrique
#define NETWORK_FREQ_F		60.0f						// [Hz] Frequence nominale du reseau electrique (float)

#define DELAY_ARRAY_SIZE 	(F_CALC/(4*NETWORK_FREQ))	// Taille du tableau pour cr�er un d�lai de signal �quivalent � pi/4 [rad] � la fr�quence de NETWORK_FREQ
#define SIN_TABLE_SIZE		200							// Taille de la LUT de la fct. sinus (max. 254)
#define MAX_FREQ_DEVIATION	6.0f							// [Hz] D�viation maximale � partir de la fr�quence nominale permise pour le verouillage
#define PLL_KI				2500.0f						// Gain int�gral du compensateur PI
#define	PLL_KP				50.0f							// Gain proportionnel du compensateur PI

#define PI_F			3.141592f					// Pi as float, not double.


/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct {

	//volatile float I_Q;
	volatile float PI_ctrl_integ_term; /* Integral term of the PLL's PI controller when in closed loop. Meaningless when open loop.*/
	volatile float w_est; /* [rad/s] angular velocity of the network (frequency times 2*pi) */
	volatile float theta_est; /* [rad] Angle of the b_bete sine wave */
	volatile float b_beta; /* A sine wave [-1, 1], in sync with the grid if the PLL is in closed-loop, free running at a fixed frequency if open-loop.*/
	volatile float freq_Hz; /* Frequency value, low pass filtered and converted to hertz*/
	volatile bool zero_crossing; /* True during a sample following a zero crossing (positive-going or negative-going) */

} PLL_state_variables_t;



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables to be made available externally                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern PLL_state_variables_t PLL;

extern float sin_table[];

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                            	   */
/*---------------------------------------------------------------------------------------------------------*/

void init_sin_table(float*, uint8_t);	// Initialise le tableau de r�f�rence pour la fonction sinus (Look-up table, LUT)

void PLL_main(void);			// Fonction principale du PLL. Prend en entr�e une onde sinusoidale normalis�e � une amplitude d'environ 1
float delay_line(float);		// Retourne le signal donn� en entr�e avec un d�lai de DELAY_ARRAY_SIZE �chantillons.
float sin_LUT(float, float*); 	// Returns the sine of an angle if it is between 0 and 2*pi. Returns 0 otherwise.
float cos_LUT(float, float*); 	// Returns the cosine of an angle if it is between 0 and 2*pi. Returns 0 otherwise. Takes the same sin_table as sin_LUT for an input

#endif /* PLL_H_ */
