/*
 * sys.c
 *
 *  Created on: Apr 4, 2022
 *      Author: Hugo Boyce
 */

#include "sys.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions definitions                                                                      	   */
/*---------------------------------------------------------------------------------------------------------*/
void delay_ms(uint32_t delay){ /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/

	uint64_t end_time = g_SysTickIntCnt + ((uint64_t)delay);

	while(g_SysTickIntCnt <= end_time){ /* As long as the end time is not reached*/
		/* Do nothing*/
	}
}
