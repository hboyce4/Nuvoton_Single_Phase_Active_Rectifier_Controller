/*
 * analog.h
 *
 *  Created on: Feb 6, 2021
 *      Author: Hugo Boyce
 */

#ifndef ANALOG_H_
#define ANALOG_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "NuMicro.h"
#include "main.h"
#include "constants.h"
#include "PLL.h"
#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

//#define EADC_DMA_CHANNEL 2
#define EADC_OVERSAMPLING_NUMBER 16 /* MAX = 16 !!! 2^EADC_SHIFT_FOR_OVERSAMPLING_DIVISION = EADC_OVERSAMPLING_NUMBER*/
#define EADC_SHIFT_FOR_OVERSAMPLING_DIVISION 4  /* MAX = 4 !!! Log base 2 of EADC_OVERSAMPLING_NUMBER equals EADC_SHIFT_FOR_OVERSAMPLING_DIVISION */
#define EADC_TOTAL_CHANNELS 8
#define VREF_VOLTAGE 3.297 /*[V] Voltage of the chip's analog voltage reference (Vref)*/


/*ADC Channels*/
#define VBUS_PLUS_CHANNEL 6
#define VBUS_MINUS_CHANNEL 5
#define V_AC_CHANNEL 4
#define I_PV_CHANNEL 1


/****************************/
/*	Input signals gains and offsets	*/
/****************************/

// Gain and offset for v bus +
#define VBUS_PLUS_GAIN 0.100202 /* [V/V] Gain of the analog front end for the bus voltages.*/
#define VBUS_PLUS_OFFSET 0.163803 /* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v bus -
#define VBUS_MINUS_GAIN 0.096989 /* [V/V] Gain of the analog front end for the bus voltages. Do not put negative sign here.*/
#define VBUS_MINUS_OFFSET 0.061594 /* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v AC (volts on alternating current side)
#define V_AC_GAIN 0.058824 			/* [V/V] Gain of the AC voltage front end*/
#define V_AC_OFFSET 28.09 			/* [V] Offset of the AC voltage front end*/

// Gain and offset for i PV (current, process value)
#define I_PV_GAIN 0.033				/* [V/A] Gain of the current process value (actual current) */
#define I_PV_OFFSET 50.5				/* [A] Offset of the current process value (actual current) */


/********************************/
/*	Output gains and offsets	*/
/********************************/

// Gain and offset for i SP (current, setpoint)
#define I_SP_GAIN 0.033				/* [V/A] Gain of the current setpoint command */
#define I_SP_OFFSET (RES_11BIT/2)	/* [LSB] Offset of the current setpoint command */

// Gain and offset for d FF (duty cycle, feedforward signal)
#define D_FF_GAIN 1					/*  Gain of the duty cycle feed forward value */
#define D_FF_OFFSET (RES_11BIT/2)	/* [LSB] Offset of the duty cycle feed forward value */

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables to be made available externally                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern volatile uint16_t ADC_acq_buff[EADC_TOTAL_CHANNELS];
extern volatile uint8_t ADC_acq_count;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void run_ADC_cal(void);
//void init_ADC_DMA(void);
//void reload_ADC_DMA(void);

void process_ADC(void);

void convert_to_float(void);
void convert_to_int_write_analog(void);


#endif /* ANALOG_H_ */
