/*
 * interrupt.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Hugo Boyce
 */

#include "interrupt.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint64_t g_SysTickIntCnt = 0;

volatile bool g_UI_new_frame_tick = false;

volatile bool g_Interrupt_real_time_fault = false;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void){ /* Very high frequency interrupt. Keep very light!!! */

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */

	uint8_t channel;
	for(channel = 0; channel < EADC_TOTAL_CHANNELS;channel++){/* Acquire latest data from ADC, filtering out status bits*/

		ADC_acq_buff[channel] += (uint16_t)(EADC->DAT[channel]);/* Only keep the lowest 16 bits of the register*/

	}

	ADC_acq_count--;/* Decrease the number of acquisitions left to make*/

    if(!ADC_acq_count){/* If zero acquisitions left to do */
    	NVIC_DisableIRQ(EADC00_IRQn); /* Stop the interrupt */

    }

}

void TMR1_IRQHandler(void){ /*High frequency interrupt (F_CALC). Keep light!!! */

    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag, or else interrupt is executed forever */
        TIMER_ClearIntFlag(TIMER1);

        //PA->DOUT &= ~(BIT12);//Turn ON green LED for timing measurements
        PA12 = 0;

        /* Begin control loop iteration */
		process_ADC(); // Service ADC routine
        convert_to_float(); // Service analog input values (convert ints from ADC to float for maths
		PLL_main(); // Service the PLL. Needs up-to-date analog input values.
		inverter_control_main(); // Service the inverter. Needs up-to-date PLL and analog input values.
		convert_to_int_write_analog(); // Convert the calculated output values to int and output them

		if(TIMER_GetIntFlag(TIMER1) == 1){/* If timer interrupt retrigerred before it's finished, we have a real time fault (cannot finish task on time)*/
			g_Interrupt_real_time_fault = true;
		}

		//PA->DOUT |= BIT12;//Turn OFF green LED for timing measurements
		PA12 = 1;
    }


}

void SysTick_Handler(void)	// Every millisecond (Medium frequency).
{
    g_SysTickIntCnt++;

    PA13 = 0; // Turn on amber LED

    inverter_medium_freq_task();

    ADC_calc_averages();

	static uint16_t UI_refresh_counter = 0;
	if(!UI_refresh_counter){
		UI_refresh_counter = UI_FRAME_INTERVAL_MS;
		g_UI_new_frame_tick = true;
	}
	UI_refresh_counter--;

	// TODO: Raise fault if retrigger before end of task

	PA13 = 1; // Turn off amber LED

}

void PDMA_IRQHandler(void){


    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x2){     /* If the interrupt is "Transfer Done" */

        if (PDMA_GET_TD_STS(PDMA) & (1 << UART2_TX_DMA_CHANNEL)){ /* If the transfer that was completed is the UART2_TX_DMA_CHANNEL*/
            PDMA_CLR_TD_FLAG(PDMA,1 << UART2_TX_DMA_CHANNEL);

            UART_DMA_Xfer_t temp;

            		if (!pop_UART2(&temp)){	// If popping a transfer "job" from the buffer succeeds
            			start_UART2_DMA_Xfer(temp); // Start the transfer

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


