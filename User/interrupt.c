/*
 * interrupt.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Hugo Boyce
 */

#include "interrupt.h"
#include "NuMicro.h"
#include "timers.h"
#include "analog.h"
#include "measurement.h"
#include "UART_over_DMA.h"
#include "sys.h"

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

	ADC_process_interrupt();
}

void TMR1_IRQHandler(void){ /*High frequency interrupt (F_CALC). Keep light!!! */

    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag, or else interrupt is executed forever */
        TIMER_ClearIntFlag(TIMER1);

        //PA->DOUT &= ~(BIT12);//Turn ON green LED for timing measurements
        GREEN_LED_PIN = 0;

        /* Begin control loop iteration */
		Measurement_process_oversampling(); // Service ADC routine
        Measurement_convert_to_float(); // Service analog input values (convert ints from ADC to float for maths
        Measurement_calc_averages_short_term();// Calculate the float average values

		inverter_control_main(); // Service the inverter. Needs up-to-date PLL and analog input values.

		if(autozero_state == AUTOZERO_DONE || autozero_state == AUTOZERO_STANDBY){ // Unless the autozero is in progress
			Measurement_convert_to_int_and_write(); // Convert the calculated output values to int and output them
		}

		if(TIMER_GetIntFlag(TIMER1) == 1){/* If timer interrupt retrigerred before it's finished, we have a real time fault (cannot finish task on time)*/
			g_Interrupt_real_time_fault = true;
		}

		//PA->DOUT |= BIT12;//Turn OFF green LED for timing measurements
		GREEN_LED_PIN = 1;
    }


}

void SysTick_Handler(void)	// Every millisecond (Medium frequency).
{
    g_SysTickIntCnt++;

    PA13 = 0; // Turn on amber LED

    inverter_medium_freq_task();

    Measurement_calc_averages_longterm();

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

    UART_DMA_process_interrupt();
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



