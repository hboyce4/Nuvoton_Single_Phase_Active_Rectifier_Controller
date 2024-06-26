/*
 * persistent_data.h
 *
 *  Created on: Jul 25, 2022
 *      Author: Hugo Boyce
 */

#ifndef PERSISTENT_DATA_H_
#define PERSISTENT_DATA_H_

#define UID_SIZE 3 /* Unique ID length is 96 bits or 3 * 32bits */
#define SOME_RANDOM_OFFSET 5





extern uint32_t g_u32UID[UID_SIZE];


void PD_Init(void);
void PD_SaveConfig(void);
void PD_LoadConfig(void);
//void PD_ClearConfig(void);

uint32_t PD_ComputeConfigChecksum(uint32_t start, uint32_t len);

#endif /* PERSISTENT_DATA_H_ */
