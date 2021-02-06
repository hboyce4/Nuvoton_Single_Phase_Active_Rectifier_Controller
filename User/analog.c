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


int32_t ADC_raw_val_buff[EADC_OVERSAMPLING_NUMBER*EADC_TOTAL_CHANNELS];

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
    //NVIC_EnableIRQ(EADC00_IRQn);

    /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
    //g_u32AdcIntFlag = 0;
    //g_u32COVNUMFlag = 0;
    EADC_START_CONV(EADC, BIT7);

    //__WFI();

    /* Disable the sample module 7 interrupt */
    //EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);
}

void run_ADC_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk|EADC_CALCTL_CALSEL_Msk;/* Set CALSTART to start calibration. Set CALSEL so it doesn't try to load user calibration word*/
	/* EADCDIV must be zero to run calibration */
}

void convert_from_ADC(void){

	int8_t channel;
	for(channel = 0; channel < EADC_TOTAL_CHANNELS;channel++){/* Acquire latest data from ADC, filtering out status bits*/

		ADC_raw_val_buff[channel]= (int32_t)((EADC->DAT[channel])&0xFFFF);/* Only keep the lowest 16 bits of the register*/

	}

	/* Vbus plus */
	inverter_state_variables.V_DC_plus = ((float)ADC_raw_val_buff[VBUS_PLUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES*VBUS_GAIN));
	/* Vbus minus */
	inverter_state_variables.V_DC_minus = -((float)ADC_raw_val_buff[VBUS_MINUS_CHANNEL])*(VREF_VOLTAGE/(ADC_RES*VBUS_GAIN));
	/* Vbus total */
	inverter_state_variables.V_DC_total = inverter_state_variables.V_DC_plus + (-1*inverter_state_variables.V_DC_minus);
	/* Vbus diff */
	inverter_state_variables.V_DC_diff = inverter_state_variables.V_DC_plus - inverter_state_variables.V_DC_minus;
	/* V AC */
	inverter_state_variables.v_AC = ((float)ADC_raw_val_buff[V_AC_CHANNEL])*(VREF_VOLTAGE/(ADC_RES*V_AC_GAIN))-V_AC_OFFSET;
	/* V AC normalized*/
	inverter_state_variables.v_AC_n = inverter_state_variables.v_AC*(1/(V_AC_NOMINAL_RMS_VALUE*M_SQRT2)); /* needs math.h */
}

void convert_to_DAC(void){


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
