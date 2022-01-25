/*
 * analog.c
 *
 *  Created on: Feb 6, 2021
 *      Author: Hugo Boyce
 */

#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


volatile int32_t ADC_raw_val[EADC_TOTAL_CHANNELS];
volatile uint16_t ADC_acq_buff[EADC_TOTAL_CHANNELS];
volatile uint8_t ADC_acq_count;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/


void init_ADC(void){


    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 0);
    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 1, EADC_ADINT0_TRIGGER, 1);
    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 2, EADC_ADINT0_TRIGGER, 2);
    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 3, EADC_ADINT0_TRIGGER, 3);
    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 4);
    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 5);
    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 6);
    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 7);

    /* Clear the A/D ADINT0 interrupt flag for safety */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 7 interrupt */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module  A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);//Enable sample module 7 interrupt.
    NVIC_SetPriority(EADC00_IRQn, ADC_INT_PRIORITY);

    /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
    //g_u32AdcIntFlag = 0;
    //g_u32COVNUMFlag = 0;
    EADC_START_CONV(EADC, BIT7);

#ifdef DEBUG_TIMINGS


#endif
    //__WFI();

    /* Disable the sample module 7 interrupt */
    //EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);
}

void init_DAC(void){

	/* Set the software trigger DAC and enable D/A converter */
	//DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);
	DAC_Open(DAC1, 0, DAC_SOFTWARE_TRIGGER);

	//DAC0->CTL |= BIT8; /* Set Bit 8 of the control register to disable the output buffer*/
	//DAC1->CTL |= BIT8; /* Set Bit 8 of the control register to disable the output buffer*/

	/* Enable DAC to work in group mode, once group mode enabled, DAC1 is configured by DAC0 registers */
	//DAC_ENABLE_GROUP_MODE(DAC0);

    /* The DAC conversion settling time is 1us */
    //DAC_SetDelayTime(DAC0, 1);
    DAC_SetDelayTime(DAC1, 6);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC1, 0, 0x400);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC1, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC1, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC1);
}

void run_ADC_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk|EADC_CALCTL_CALSEL_Msk;/* Set CALSTART to start calibration. Set CALSEL so it doesn't try to load user calibration word*/
	/* EADCDIV must be zero to run calibration */
}

void process_ADC(void){

	/* Divide,copy and reset */
	uint8_t channel;
	for(channel = 0; channel < EADC_TOTAL_CHANNELS;channel++){/* For every channel */

		/* this step could be skipped by integrating this calculation in the float conversion step*/
		ADC_raw_val[channel]= (ADC_acq_buff[channel] >> EADC_SHIFT_FOR_OVERSAMPLING_DIVISION); /* Copy the acquisition buffer, dividing by the number of oversamples. */

		ADC_acq_buff[channel] = 0;
	}

	/* Start ADC acquisition batch*/
	ADC_acq_count = EADC_OVERSAMPLING_NUMBER;

	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	NVIC_EnableIRQ(EADC00_IRQn);

}


void convert_to_float(void){

	/* Vbus plus */
	inverter.V_DC_plus = ((float)ADC_raw_val[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*VBUS_GAIN));
	/* Vbus minus */
	inverter.V_DC_minus = -((float)ADC_raw_val[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*VBUS_GAIN));
	/* Vbus total */
	inverter.V_DC_total = inverter.V_DC_plus - inverter.V_DC_minus;
	/* Vbus diff */
	inverter.V_DC_diff = inverter.V_DC_plus + inverter.V_DC_minus;
	/* V AC */
	inverter.v_AC = ((float)ADC_raw_val[V_AC_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*V_AC_GAIN))-V_AC_OFFSET;
	/* V AC normalized*/
	inverter.v_AC_n = inverter.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*M_SQRT2)); /* needs math.h */
	/* Current process value (actual amperage) */
	inverter.i_PV = ((float)ADC_raw_val[I_PV_CHANNEL])*(VREF_VOLTAGE/(RES_12BIT*I_PV_GAIN))-I_PV_OFFSET;
}



void convert_to_int_write_analog(void){

	int32_t i_sp_val, d_ff_val;

	/* Convert the current setpoint from float to int*/
	i_sp_val = 	(int32_t)(inverter.i_SP*I_SP_GAIN*(RES_11BIT/VREF_VOLTAGE));
	i_sp_val += I_SP_OFFSET; /* signal centered around I_SP_OFFSET */

	/* Convert the duty cycle feedforward value from int to float */
	d_ff_val = (int32_t)((inverter.d_feedforward-0.5)*D_FF_GAIN*(RES_11BIT/VREF_VOLTAGE)); /* add -0.5 shift to make signal between -0.5 and 0.5 */
	d_ff_val += D_FF_OFFSET; /* Centered around D_FF_OFFSET */

#ifdef PWM_DAC

	EPWM1->CMPDAT[2] = (uint32_t)i_sp_val;/* DAC1 pin */
	EPWM1->CMPDAT[3] = (uint32_t)d_ff_val;/* DAC0 pin */
#else
	 DAC_WRITE_DATA(DAC1, 0, i_sp_val);
	 DAC_START_CONV(DAC1);
#endif


}


//void init_ADC_DMA(void){
//	   /* Configure PDMA peripheral mode form EADC to memory */
//
//			PDMA->CHCTL = (0b1 << EADC_DMA_CHANNEL);			/* The channel we want to use is enabled in the CHCTL register */
//
//		    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk)|(PDMA_EADC0_RX << PDMA_REQSEL0_3_REQSRC2_Pos); /* connect channel to EADC0_TX. Channel 2 is used */
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].CTL = PDMA_REQ_SINGLE|PDMA_SAR_FIX|PDMA_WIDTH_16;	/* 0b1 for TX type = single transfer*/
//														/* 0b11 for no increment of source address (fixed address)*/
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].CTL = 	(PDMA->DSCT[EADC_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((((EADC_OVERSAMPLING_NUMBER+1)*EADC_TOTAL_CHANNELS)-1) << PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_OP_BASIC; /* OR the string length in the register and set the operating mode from idle to basic*/
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].DA = ((uint32_t)&ADC_raw_val_buff); 	/* Destination address is the storage struct */
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].SA = ((uint32_t)&(EADC->CURDAT)); 		/* The constant source address is the EADC CURDAT register */
//
//		    EADC->INTEN |= (UART_INTEN_TXPDMAEN_Msk /*| UART_INTEN_RDAIEN_Msk*/); 		/* Bit TXPDMAEN is set to one enable PDMA requests. RDAIEN is set to generate interrupt on character receive*/
//		    /* The fist time the DMA is used, the UARTn->INTEN write must absolutely be done at the end */
//
//		    //delay_ms(10); /* Make sure the init string is transferred before anything else happens*/
//
//		    EADC_ENABLE_SAMPLE_MODULE_PDMA();
//
//		    PDMA_EnableInt(PDMA,EADC_DMA_CHANNEL, PDMA_INT_TRANS_DONE);
//		    NVIC_EnableIRQ(PDMA_IRQn);
//
//		    NVIC_SetPriority(PDMA_IRQn, DMA_INTERRUPT_PRIORITY);
//
//
//}

//void start_ADC_acq(void){
//
//	/*Reload DMA*/
//	PDMA->DSCT[EADC_DMA_CHANNEL].CTL = 	(PDMA->DSCT[EADC_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|
//			((((EADC_OVERSAMPLING_NUMBER+1)*EADC_TOTAL_CHANNELS)-1) << PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_OP_BASIC; /* OR the string length in the register and set the operating mode from idle to basic*/
//
//	/*Enable ADINT0 by sample module 7 for self triggering in a loop*/
//	EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);//Enable sample module 7 interrupt.
//
//	/*Trigger sample module 7 to */
//
//}
