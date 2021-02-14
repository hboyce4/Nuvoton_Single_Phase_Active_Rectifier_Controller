/*
 * interrupt.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Hugo Boyce
 */

#include "interrupt.h"


void PDMA_IRQHandler(void){

#ifdef TIMING_DEBUG
        PH->DOUT &= ~(BIT3);//Timing measurements
#endif

    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x2){     /* If the interrupt is "Transfer Done" */

        if (PDMA_GET_TD_STS(PDMA) & (1 << UART1_TX_DMA_CHANNEL)){ /* If the transfer that was completed is the UART1_TX_DMA_CHANNEL*/
            PDMA_CLR_TD_FLAG(PDMA,1 << UART1_TX_DMA_CHANNEL);

            UART_DMA_Xfer_t temp;

            		if (!pop_UART1(&temp)){	// If popping a transfer "job" from the buffer succeeds
            			start_UART1_DMA_Xfer(temp); // Start the transfer

            		}/* else, popping from the buffer didn't not succeed, it means there's no transfer jobs left */


//        }else if(PDMA_GET_TD_STS(PDMA) & (1 << EADC_DMA_CHANNEL)){
//        	PDMA_CLR_TD_FLAG(PDMA,1 << EADC_DMA_CHANNEL);
//
//        	/* EADC Transfer complete */
//
//
        }else{
        	printf("PDMA int. from unknown ch.!!\n");
        	PDMA_CLR_TD_FLAG(PDMA,PDMA_GET_TD_STS(PDMA));
        }
    }else if (status & 0x1){   /* abort */

        printf("PDMA target abort int.!!\n");
        if (PDMA_GET_ABORT_STS(PDMA) & 0x4)
        PDMA_CLR_ABORT_FLAG(PDMA,PDMA_GET_ABORT_STS(PDMA));

    }else if (status & 0x300){	/* channel 2 timeout */

        printf(" PDMA timeout int.!!\n");

    }else{
        printf("unknown PDMA int.!!\n");
    }

#ifdef TIMING_DEBUG
		PH->DOUT |= BIT3;	//Timing measurements
#endif

}

void TMR1_IRQHandler(void){

    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag, or else interrupt is executed forever */
        TIMER_ClearIntFlag(TIMER1);


        /* Begin control loop iteration */
#ifdef TIMING_DEBUG
        PH->DOUT &= ~(BIT1);//Timing measurements
#endif
		process_ADC();
        convert_to_float();
		PLL_main();
		inverter_control_main();
		convert_to_int_write_analog();
#ifdef TIMING_DEBUG
		PH->DOUT |= BIT1;	//Timing measurements
#endif
    }


}

void EADC00_IRQHandler(void){ /* Very high frequency interrupt. Keep very light!!! */

#ifdef TIMING_DEBUG
	PH->DOUT &= ~(BIT2);//Timing measurements
#endif
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */

	uint8_t channel;
	for(channel = 0; channel < EADC_TOTAL_CHANNELS;channel++){/* Acquire latest data from ADC, filtering out status bits*/

		ADC_acq_buff[channel] += (uint16_t)(EADC->DAT[channel]);/* Only keep the lowest 16 bits of the register*/

	}

	ADC_acq_count--;/* Decrease the number of acquisitions left to make*/

    if(!ADC_acq_count){/* If zero acquisitions left to do */
    	NVIC_DisableIRQ(EADC00_IRQn); /* Stop the interrupt */

    }
#ifdef TIMING_DEBUG
    PH->DOUT |= BIT2;	//Timing measurements
#endif
}

#ifndef PWM_DAC
void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC1, 0)){
    	/* Clear flag */
    	 DAC_CLR_INT_FLAG(DAC1, 0);

    }
    return;
}
#endif

//void UART1_IRQHandler(void)
//{
//    uint32_t u32DAT;
//    uint32_t u32IntSts = UART1->INTSTS;
//
//    if(u32IntSts & UART_INTSTS_RDAIF_Msk)
//    {
//        u32DAT = UART1->DAT; // read out data
//        printf("\nReceived character '0x%x' \n", u32DAT);
//
//    }
//}
