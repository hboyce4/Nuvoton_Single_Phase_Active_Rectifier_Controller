/*
 * inverter_control.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Hugo Boyce
 */

#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

inverter_state_variables_t inverter_state_variables;
inverter_state_safety_t inverter_state_safety;
inverter_setpoints_t inverter_setpoints;
inverter_limits_t inverter_limits;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void init_inverter_control(void){

    /*Using EPWM1, */
    /* Begin to output the carrier waveform for the analog hardware PWModulator on PB.14 (pin 133 on the M487JIDAE) */
	PB->SLEWCTL |= (GPIO_SLEWCTL_HIGH << 2*14); /*Set PB14 to "High" slew rate.*/
	/* For some reason "High" mode seems faster than "Fast" mode. Most likely it's just my probing setup (signal reflexions and such) */
	EPWM_ConfigOutputChannel(EPWM1, 1, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480. Use the defined carrier freq, with 50% duty cycle */
	EPWM1->POEN |= 0b10; /* Enable CH1 (set 1 at bit position 1)*/
	EPWM1->CNTEN |= 0b10; /* Start the counter (set 1 at bit position 1)*/

	delay_ms(10); /* Give the analog modulator some time to stabilize */

	/* Start timer 1. Triggers the control loop interrupt */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, F_CALC); /* Set the interrupt frequency as F_CALC*/
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_SetPriority(TMR1_IRQn, TMR1_INT_PRIORITY);
    TIMER_Start(TIMER1);

}
