/*
 * interrupt.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Hugo Boyce
 */

#include "interrupt.h"


void PDMA_IRQHandler(void){

    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x2){     /* If the interrupt is "Transfer Done" */

        if (PDMA_GET_TD_STS(PDMA) & (1 << UART1_TX_DMA_CHANNEL)){ /* If the transfer that was completed is the UART1_TX_DMA_CHANNEL*/
            PDMA_CLR_TD_FLAG(PDMA,1 << UART1_TX_DMA_CHANNEL);

            UART_DMA_Xfer_t temp;

            		if (!pop_UART1(&temp)){	// If popping a transfer "job" from the buffer succeeds
            			start_UART1_DMA_Xfer(temp); // Start the transfer

            		}/* else, popping from the buffer didn't not succeed, it means there's no transfer jobs left */


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
