/*
 * persistent_data.c
 *
 *  Created on: Jul 25, 2022
 *      Author: Hugo Boyce
 */

#include <stdbool.h>
#include <stdint.h>
#include "NuMicro.h"
#include "persistent_data.h"
#include "analog.h"
#include "measurement.h"
//#include "CAN_message.h"
//#include "errors.h"

/* The code assumes everything fits on a single page! (4kB ) */
#define PD_ANALOG_CHANNEL_OFFSET 0x40 /* 64 bytes or 16 floats per channel */
#define PD_ENABLE_OFFSET 0
#define PD_SENSOR_TYPE_OFFSET 4
#define PD_FIELD_UNIT_OFFSET 8
#define PD_PROCESS_UNIT_OFFSET 12
#define PD_BIAS_RESISTOR_OFFSET 16
#define PD_NTC_R_ZERO_OFFSET 20
#define PD_NTC_BETA_OFFSET 24
#define PD_VOLTAGE_GAIN_OFFSET 28
#define PD_SENSOR_GAIN_OFFSET 32

#define PD_CAN_SPEED_OFFSET 0x300
#define PD_CAN_NODE_ID_OFFSET 0x304

#define CHECKSUM_OFFSET 0x400 /* Comes after everything else*/


uint32_t g_u32UID[UID_SIZE];

bool g_Error_InvalidConfig = true;


void PD_Init(void){

	SYS_UnlockReg();
	FMC_Open();       /* Enable FMC ISP function   */


	/* Get UID*/
    uint8_t i;
    for(i = 0; i < UID_SIZE; i++){
    	g_u32UID[i] = FMC_ReadUID(i);
    }

    /* Get data flash base address */
    uint32_t DataFlashBaseAdress = FMC_ReadDataFlashBaseAddr();

    if((FMC_Read(FMC_CONFIG_BASE) & 0x01) ||  DataFlashBaseAdress == 0xFFFFFFFF){/* If the DFEN bit if the CONFIG says "data flash disabled" or the data flash base address is all "F"s */

    	printf("No Data Flash Allocated!!\n");
    	printf("Allocate some using Nuvoton ICP Prog. Tool\n");
    }else{

    	uint32_t checksum;
    	checksum = PD_ComputeConfigChecksum(DataFlashBaseAdress, CHECKSUM_OFFSET);/* Compute the config's checksum */

    	if(checksum == inpw(DataFlashBaseAdress + CHECKSUM_OFFSET)){/* If the computed checksum is equal to the checksum stored in memory */
    		printf("Config CRC32 OK\n");
    		printf("Loading config from memory...\n");
    		PD_LoadConfig();
    		//Error_Clear(ERROR_CORRUPTED_CONFIG);

    	}else{

    		printf("Computed CRC32:%X\n", checksum);
    		printf("CRC32 found in memory: %X\n", inpw(DataFlashBaseAdress + CHECKSUM_OFFSET));
    		printf("Config CRC32 invalid!!!\n");
    		printf("Using default config!\n");
    	}


    }


    FMC_Close();
    SYS_LockReg();
}

void PD_SaveConfig(void){

	SYS_UnlockReg();
	FMC_Open();       /* Enable FMC ISP function   */
	FMC_ENABLE_AP_UPDATE();

	uint32_t DataFlashBaseAdress = FMC_ReadDataFlashBaseAddr();

	if(!(FMC_Read(FMC_CONFIG_BASE) & 0x01) && (DataFlashBaseAdress != 0xFFFFFFFF)){ /* If the data flash base address is not set to the obviously invalid value... */
		/* .. and the DFEN bit is set to "data flash enabled" */

		/* Erase */
		FMC_Erase(DataFlashBaseAdress);/* Erase one page, starting at DFBA. */

		/* Verify erase */
		if(!(FMC_CheckAllOne(DataFlashBaseAdress, 1 * FMC_FLASH_PAGE_SIZE) == READ_ALLONE_YES)){/* Check all one on one page*/
			printf("Memory erase failed! Cannot write.\n");
		}else{

			/* Write */
			uint8_t i;
			uint32_t temp;
			uint8_t *struct_base_ptr; //(one) byte pointer
			struct_base_ptr = (uint8_t*)&measurement_offsets;
			for(i = 0; i < (sizeof(measurement_offsets)/4); i++){ //Maximum size is CHECKSUM_OFFSET bytes
				memcpy(&temp, struct_base_ptr + (i*4), 4);
				FMC_Write(DataFlashBaseAdress + (i*4), temp);
			}

			if(sizeof(measurement_offsets) % 4){
				temp = 0;
				memcpy(&temp,struct_base_ptr + (i*4), sizeof(measurement_offsets) % 4);
				FMC_Write(DataFlashBaseAdress + (i*4), temp);
			}

			uint32_t CRC32 = PD_ComputeConfigChecksum(DataFlashBaseAdress, CHECKSUM_OFFSET);

			printf("Config. CRC32: %X\n", CRC32);

			FMC_Write(DataFlashBaseAdress + CHECKSUM_OFFSET, CRC32);

		}

	}else{
		printf("No Data Flash Allocated!!\n");
	}

	FMC_Close();
	SYS_LockReg();
}

uint32_t PD_ComputeConfigChecksum(uint32_t start, uint32_t len){

	//uint32_t CRC32;
	//CRC32 = FMC_GetChkSum(DataFlashBaseAdress, CHECKSUM_OFFSET);/* Getting the checksum with the FMC is annoying because it only does 4K chunks */

	/* Configure CRC controller for CRC-CRC32 mode. Taken from Nuvoton BSP V3.05.003. Don't forget to enable clock! */
	CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);
	/* Start to execute "manual" CRC-CRC32 operation. Could be done with PDMA but we're not in a hurry. */
	uint32_t addr;
	for(addr = start; addr < start + len; addr+=4)
	{
		CRC_WRITE_DATA(inpw(addr));
	}
	return CRC_GetChecksum();

}


void PD_LoadConfig(void){


	uint32_t DataFlashBaseAdress = FMC->DFBA;

	memcpy((uint8_t*)&measurement_offsets,(uint8_t*)DataFlashBaseAdress, sizeof(measurement_offsets));

}


//void PD_ClearConfig(void){ /* Only handles a single page!!!*/
//
//	SYS_UnlockReg();
//	FMC_Open();       /* Enable FMC ISP function   */
//	FMC_ENABLE_AP_UPDATE();
//
//	uint32_t DataFlashBaseAdress = FMC_ReadDataFlashBaseAddr();
//
//	if(!(FMC_Read(FMC_CONFIG_BASE) & 0x01) && (DataFlashBaseAdress != 0xFFFFFFFF)){ /* If the data flash base address is not set to the obviously invalid value... */
//		/* .. and the DFEN bit is set to "data flash enabled" */
//
//
//		/* Erase */
//		FMC_Erase(DataFlashBaseAdress);/* Erase one page, starting at DFBA. */
//
//		/* Verify erase */
//		if(!(FMC_CheckAllOne(DataFlashBaseAdress, 1 * FMC_FLASH_PAGE_SIZE) == READ_ALLONE_YES)){/* Check all one on one page*/
//			printf("Memory erase failed! Cannot write.\n");
//		}
//
//	}else{
//		printf("No Data Flash Allocated!!\n");
//	}
//
//	FMC_Close();
//	SYS_LockReg();
//}
