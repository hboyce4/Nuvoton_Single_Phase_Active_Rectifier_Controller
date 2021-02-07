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
#include "PLL.h"
#include "inverter_control.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#define EADC_DMA_CHANNEL 2
#define EADC_OVERSAMPLING_NUMBER 16 /* MAX = 16 !!! 2^EADC_SHIFT_FOR_OVERSAMPLING_DIVISION = EADC_OVERSAMPLING_NUMBER*/
#define EADC_SHIFT_FOR_OVERSAMPLING_DIVISION 4  /* MAX = 4 !!! Log base 2 of EADC_OVERSAMPLING_NUMBER equals EADC_SHIFT_FOR_OVERSAMPLING_DIVISION */
#define EADC_TOTAL_CHANNELS 8
#define VREF_VOLTAGE 3.239 /*[V] Voltage of the chip's analog voltage reference (Vref)*/
#define ADC_RES 4096.0


/*ADC Channels*/
#define VBUS_PLUS_CHANNEL 0
#define VBUS_MINUS_CHANNEL 1
#define V_AC_CHANNEL 2

/* Gains */
#define VBUS_GAIN 0.1 /* [V/V] Gain of the analog front end for the bus voltages. The negative bus's front end has a gain of -VBUS_GAIN*/
#define V_AC_GAIN 0.058824 /* [V/V] Gain of the AC voltage front end*/
#define V_AC_OFFSET 28.05 /* [V] Offset of the AC voltage front end*/
#define V_AC_NOMINAL_RMS_VALUE 12 /* Nominal RMS value of the input voltage (ex.: 12V AC rms)*/



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

void init_ADC(void);
void run_ADC_cal(void);
//void init_ADC_DMA(void);
//void reload_ADC_DMA(void);

void process_ADC(void);

void convert_to_float(void);
void convert_to_DAC(void);


#endif /* ANALOG_H_ */
