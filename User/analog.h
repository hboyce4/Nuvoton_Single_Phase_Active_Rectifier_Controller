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
#define EADC_OVERSAMPLING_NUMBER 8 /* MAX = 16 !!! 2^EADC_SHIFT_FOR_OVERSAMPLING_DIVISION = EADC_OVERSAMPLING_NUMBER*/
#define EADC_SHIFT_FOR_OVERSAMPLING_DIVISION 3  /* MAX = 4 !!! Log base 2 of EADC_OVERSAMPLING_NUMBER equals EADC_SHIFT_FOR_OVERSAMPLING_DIVISION */
#define EADC_TOTAL_CHANNELS 8
#define VREF_VOLTAGE 3.297 /*[V] Voltage of the chip's analog voltage reference (Vref)*/


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
#define VBUS_PLUS_GAIN 0.100202 /* [V/V] Gain of the analog front end for the bus voltages.*/
#define VBUS_PLUS_OFFSET 0.163803 /* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v bus -
#define VBUS_MINUS_GAIN 0.096989 /* [V/V] Gain of the analog front end for the bus voltages. Do not put negative sign here.*/
#define VBUS_MINUS_OFFSET 0.061594 /* [V] Vmeasured by the MCU minus real voltage  */

// Gain and offset for v AC (volts on alternating current side)
#define V_AC_GAIN 0.058824 			/* [V/V] Gain of the AC voltage front end*/
#define V_AC_OFFSET 2050 			/* [ADC counts] Initial offset of the AC voltage front end. Will be auto-zeroed */
//#define V_AC_OFFSET 28.09 			/* [V] Offset of the AC voltage front end*/

// Gain and offset for i PV (current, process value)
#define I_PV_GAIN 0.033				/* [V/A] Gain of the current process value (actual current) */
#define I_PV_OFFSET 2053				/* [ADC counts] Initial offset of the current process value (actual current). Will be auto-zeroed */
//#define I_PV_OFFSET 50.5				/* [A] Offset of the current process value (actual current) */


/********************************/
/*	Output gains and offsets	*/
/********************************/

// Gain and offset for i SP (current, setpoint)
#define I_SP_GAIN 0.033				/* [V/A] Gain of the current setpoint command */
#define I_SP_OFFSET (DAC_RES_COUNT/2)	/* [LSB] Offset of the current setpoint command */

// Gain and offset for d FF (duty cycle, feedforward signal)
#define D_FF_GAIN 1					/*  Gain of the duty cycle feed forward value */
#define D_FF_OFFSET (DAC_RES_COUNT/2)	/* [LSB] Offset of the duty cycle feed forward value */

// Long term averages of alternating values
#define NB_SAMPLES_LONG_TERM_AVG 1000 /* Number of samples for calculating the long term average of analog inputs. Max value somewhere around 524288 (31 bit int / 12 bit ADC resolution) */


// Autozero conditions
#define VBUS_MIN_FOR_AUTOZERO 20 /* [V] Minimum voltage on the busses to perform autozero */
#define COUNT_MARGIN_FOR_AUTOZERO 1750 /* [counts] Margin, in ADC counts, to consider the signal saturated (to 3.3V or GND))*/

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum {AUTOZERO_STANDBY, AUTOZERO_WAIT_FOR_CONDITIONS, AUTOZERO_IN_PROGRESS, AUTOZERO_DONE} autozero_state_t;

typedef struct analog_inputs{ /* Process values or state variables */


	volatile float V_DC_plus;
	volatile float V_DC_minus;
	volatile float V_DC_total;
	volatile float V_DC_diff;
	volatile float v_AC;
	volatile float v_AC_n; /* v_AC normalized to a peak amplitude of one */
	volatile float i_PV;

	volatile float T_transformer;
	volatile float T_inverter;

} analog_inputs_t;



typedef struct analog_averages{ /* measured long-term averages at the inputs, for zeroing */

	volatile uint32_t v_Mid;
	volatile uint32_t v_AC;
	volatile uint32_t i_PV;

} analog_averages_t;

typedef struct analog_offsets{ /* Offsets counts for inputs and outputs*/

	// Inputs, 12 bits
	volatile uint32_t v_AC;
	volatile uint32_t i_PV;

	//Outputs, 11 bits
	volatile uint32_t i_SP;
	volatile uint32_t d_FF;

} analog_offsets_t;

/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern volatile analog_inputs_t analog_in; /* directly measured values */
extern volatile uint16_t ADC_acq_buff[EADC_TOTAL_CHANNELS];
extern volatile uint8_t ADC_acq_count;

extern volatile analog_averages_t analog_avgs;
extern volatile analog_offsets_t analog_offsets;

extern volatile autozero_state_t autozero_state; //UI needs to be able to read this

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void run_ADC_cal(void);
//void init_ADC_DMA(void);
//void reload_ADC_DMA(void);

void ADC_service_oversampling(void);

void ADC_convert_to_float(void);
void convert_to_int_write_analog(void);
void DAC_write_i_SP(uint32_t);

void ADC_calc_averages(void);
//void ADC_autozero(void);

void analog_autozero(void);
bool analog_check_autozero_conditions_ok(void);

#endif /* ANALOG_H_ */
