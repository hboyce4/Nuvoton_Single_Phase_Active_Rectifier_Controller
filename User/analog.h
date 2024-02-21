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
#include "measurement.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

//#define EADC_DMA_CHANNEL 2
#define EADC_OVERSAMPLING_NUMBER 8 /* MAX = 16 !!! 2^EADC_SHIFT_FOR_OVERSAMPLING_DIVISION = EADC_OVERSAMPLING_NUMBER*/
#define EADC_SHIFT_FOR_OVERSAMPLING_DIVISION 3  /* MAX = 4 !!! Log base 2 of EADC_OVERSAMPLING_NUMBER equals EADC_SHIFT_FOR_OVERSAMPLING_DIVISION */
#define EADC_FIRST_CHANNEL 1
#define EADC_LAST_CHANNEL 7
#define VREF_VOLTAGE 3.297f /*[V] Voltage of the chip's analog voltage reference (Vref)*/


/*ADC Channels*/
#define V_MID_CHANNEL 7

#define VBUS_PLUS_CHANNEL 6
#define VBUS_MINUS_CHANNEL 5

#define V_AC_CHANNEL 4

#define PCB_TEMP_CHANNEL 3
#define XFORMER_TEMP_CHANNEL 2

#define I_PV_CHANNEL 1
#define D_COMP_CHANNEL 0


/****************************/
/*	Input signals gains and offsets	*/
/****************************/

// Gain and offset for v bus +
#define VBUS_PLUS_GAIN 0.100202f 	/* [V/V] Gain of the analog front end for the bus voltages.*/
#define VBUS_PLUS_OFFSET 0.163803f 	/* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v bus -
#define VBUS_MINUS_GAIN 0.096989f 	/* [V/V] Gain of the analog front end for the bus voltages. Do not put negative sign here.*/
#define VBUS_MINUS_OFFSET 0.061594f /* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v AC (volts on alternating current side)
#define V_AC_GAIN 0.058824f 		/* [V/V] Gain of the AC voltage front end*/
#define V_AC_OFFSET 2050 			/* [ADC counts] Initial offset of the AC voltage front end. Will be auto-zeroed */
//#define V_AC_OFFSET 28.09 		/* [V] Offset of the AC voltage front end*/

// Gain and offset for i PV (current, process value)
#define I_PV_GAIN 0.033f			/* [V/A] Gain of the current process value (actual current) */
#define I_PV_OFFSET 2053			/* [ADC counts] Initial offset of the current process value (actual current). Will be auto-zeroed */
//#define I_PV_OFFSET 50.5			/* [A] Offset of the current process value (actual current) */


/********************************/
/*	Output gains and offsets	*/
/********************************/

// Gain and offset for i SP (current, setpoint)
#define I_SP_GAIN 0.033f			/* [V/A] Gain of the current setpoint command */
#define I_SP_OFFSET (DAC_RES_COUNT/2)/* [LSB] Offset of the current setpoint command */

// Gain and offset for d FF (duty cycle, feedforward signal)
#define D_FF_GAIN 1.0f				/*  Gain of the duty cycle feed forward value */
#define D_FF_OFFSET 1014			/* [LSB] Offset of the duty cycle feed forward value. Theoretically DAC_RES_COUNT/2 */

// Long term averages of alternating values
#define NB_SAMPLES_LONG_TERM_AVG 1000 /* Number of samples for calculating the long term average of analog inputs. Max value somewhere around 524288 (31 bit int / 12 bit ADC resolution) */

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                               */
/*---------------------------------------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern volatile measurement_inputs_t measurements_in; /* directly measured values */
extern volatile uint32_t ADC_raw_val[EADC_LAST_CHANNEL+1];
extern volatile uint16_t ADC_acq_buff[EADC_LAST_CHANNEL+1];
extern volatile uint8_t ADC_acq_count;

extern volatile measurement_averages_t measurement_avgs;
extern volatile measurement_offsets_t measurement_offsets;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void ADC_run_cal(void);
//void init_ADC_DMA(void);
//void reload_ADC_DMA(void);

void ADC_process_interrupt(void);

void DAC_write_i_SP(uint32_t);

void DAC_write_d_FF(uint32_t);
uint32_t DAC_read_d_FF(void);

//void ADC_autozero(void);

#endif /* ANALOG_H_ */
