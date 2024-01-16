/*
 * UI.c
 *
 *  Created on: Jan 10, 2021
 *      Author: Hugo Boyce
 */

#include "UI.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile bool g_New_startup_from_user = 0;

uint8_t page_number = 1; /* Keeps track of the info page we're on*/

volatile uint32_t g_d_ff_zero_state = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

//TODO: Add double buffering

/* 363us in Debug, 219us in Release ( -O2 ) */
void draw_UI(int8_t row_sel, int8_t col_sel){/* 60-ish characters width*/

	/* Cursor home : \x1B[H */

    uint8_t p_line_counter = 0;

    /* Current state display */
	draw_UI_line_0_1(&p_line_counter);
	draw_UI_line_2(&p_line_counter);
	draw_UI_line_3(&p_line_counter);
	draw_UI_line_4(&p_line_counter);
	draw_UI_line_5(&p_line_counter);
	draw_UI_line_6(&p_line_counter);
	draw_UI_line_7(&p_line_counter);
	draw_UI_line_8(&p_line_counter);
	draw_UI_line_9(&p_line_counter);

	draw_UI_line_separator(&p_line_counter);
	/* Menu input*/
	draw_UI_line_A(&p_line_counter,row_sel, col_sel);
	draw_UI_line_B(&p_line_counter,row_sel, col_sel);
	draw_UI_line_C(&p_line_counter,row_sel, col_sel);
	draw_UI_line_D(&p_line_counter,row_sel, col_sel);

	draw_UI_line_separator(&p_line_counter);

	/* Last line */
	static char last_line_str[LINE_WIDTH];
	sprintf(last_line_str,"\x1B[%uA",p_line_counter);
	push_UART2((char*)last_line_str);
	/* End of last line */


	//inverter.V_DC_plus += 0.51; //debug


}

void read_user_input(int8_t* p_row_sel, int8_t* p_col_sel){

  	char user_char;
  	static uint8_t state = 0;
	uint32_t u32IntSts = UART2->INTSTS;

	if(u32IntSts & UART_INTSTS_RDAIF_Msk){/* If there is data to be read*/
		user_char = ((char)UART2->DAT); // read out data

		if (state == 0){/* If no characters have been received*/
			if(user_char == 0x1B){/* If the first character is escape*/
				state++;/* Go to the next state*/
			}

		} else if (state == 1){/* If the first character of an escape sequence (ESC) has been received*/

			if(user_char == '['){/* If the next received character is the 2nd character of an escape sequence*/
				state++;/* Go to the next state*/
			}else{
				state = 0;/* Else reset the state machine*/
			}
		} else if (state == 2){

			if(user_char == 'A'){/* Up */
				(*p_row_sel)--;/*Move the selection up*/
				if(*p_row_sel<0){/* If the selection goes past the highest row*/
					*p_row_sel = 0;/* Stay at the highest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'B'){ /* Down*/
				(*p_row_sel)++;/*Move the selection down*/
				if(*p_row_sel >= UI_MENU_NB_ROWS){/* If the selection goes past the lowest row*/
					*p_row_sel = (UI_MENU_NB_ROWS-1);/* Stay at the lowest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'C'){ /* Right */
				(*p_col_sel)++;/*Move the selection right*/
				if(*p_col_sel >= UI_MENU_NB_COLUMNS){/* If the selection goes past the lowest row*/
					*p_col_sel = (UI_MENU_NB_COLUMNS-1);/* Stay at the lowest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'D'){ /* Left*/
				(*p_col_sel)--;/*Move the selection left*/
				if(*p_col_sel<0){/* If the selection goes past the highest row*/
					*p_col_sel = 0;/* Stay at the highest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else{
				state = 0;/* Else reset the state machine*/
			}

		}

		if(user_char == '+'){/* If a '+' is received, the selected value is incremented*/
			increment_UI_value(*p_row_sel,*p_col_sel);
		}else if (user_char == '-'){/* If a '-' is received, the selected value is decremented*/
			decrement_UI_value(*p_row_sel,*p_col_sel);
		}
		//printf("\nReceived character '0x%x' \n", user_char); /* for debug */

	}
}

void increment_UI_value(int8_t row_sel, int8_t col_sel){

	switch(row_sel){

		case 0:
			if(col_sel == 0){
				if(!inverter_setpoints.inverter_active){ // if the inverter is off and we'll be turning it on in the next line
					inverter_reset_charge_errors(); // Reset the charge errors
					g_New_startup_from_user = true;
				}
				inverter_setpoints.inverter_active = true;
			}else if (col_sel == 1){
				inverter_setpoints.V_DC_total_setpoint += FLOAT_INCREMENT;
			}
			break;


		case 1:
			if(col_sel == 0){
				/*inverter_setpoints.precharge_threshold += FLOAT_INCREMENT;*/
				g_d_ff_zero_state = true;
			}else if (col_sel == 1){
				/*inverter_setpoints.V_DC_diff_setpoint += FLOAT_INCREMENT;*/
				measurement_offsets.d_FF += 1;
			}
			break;


		case 2:
			if(col_sel == 0){
				PA2 = true;
			}else if (col_sel == 1){
				PA5 = true;
			}
			break;


		case 3:

			if(col_sel == 0){
				if(page_number < NB_PAGES){
					page_number += 1;
				}
			}else if (col_sel == 1){
				autozero_state = AUTOZERO_WAIT_FOR_CONDITIONS;// Do nothing
			}
			break;

	}

}

void decrement_UI_value(int8_t row_sel, int8_t col_sel){

	switch(row_sel){

		case 0:
			if(col_sel == 0){
				inverter_setpoints.inverter_active = false;
			}else if (col_sel == 1){
				inverter_setpoints.V_DC_total_setpoint -= FLOAT_INCREMENT;
			}
			break;
		case 1:
			if(col_sel == 0){
				/*inverter_setpoints.precharge_threshold = FLOAT_INCREMENT;*/
				g_d_ff_zero_state = false;
			}else if (col_sel == 1){
				/*inverter_setpoints.V_DC_diff_setpoint -= FLOAT_INCREMENT;*/
				measurement_offsets.d_FF -= 1;
			}
			break;

		case 2:
			if(col_sel == 0){
				PA2 = false;
			}else if (col_sel == 1){
				PA5 = false;
			}
			break;

		case 3:
			if(col_sel == 0){
				if(page_number > 1){
					page_number -= 1;
				}
			}else if (col_sel == 1){
				autozero_state = AUTOZERO_STANDBY;// Do nothing
			}
			break;

	}
}


void draw_UI_line_0_1(uint8_t* p_line_counter) {
	/* Line 0 and 1*/
	static const char line_0_1_str[] =
			"\x1B[0J\n\r****ACTIVE RECTIFIER CONTROLLER V0.1****\n\r";
	/*Escape sequence to clear from cursor to end of screen*/
	*p_line_counter += 2;
	push_UART2((char*) line_0_1_str);
}

void draw_UI_line_2(uint8_t* p_line_counter) {

	static char line_2_str[LINE_WIDTH];
	static char colour_V_DC_plus[ESCAPE_SEQUENCE_LENGTH];
	static char colour_V_DC_minus[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_safety.OV_V_DC_plus) {
		/* if overvoltage */
		strcpy(colour_V_DC_plus, "\x1B[91m"); /* Red*/
	} else if (inverter_safety.UV2_V_DC_plus) {
		/* if undervoltage */
		strcpy(colour_V_DC_plus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_plus, COLOUR_DEFAULT); /* default*/
	}

	if (inverter_safety.OV_V_DC_minus) {
		/* if overvoltage */
		strcpy(colour_V_DC_minus, "\x1B[91m"); /* Red*/
	} else if (inverter_safety.UV2_V_DC_minus) {
		/* if undervoltage */
		strcpy(colour_V_DC_minus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_minus, COLOUR_DEFAULT); /* default*/
	}

	sprintf(line_2_str, "V bus +: %s%2.2f V%s \tV bus -: %s%2.2f V%s\n\r",
			colour_V_DC_plus, measurements_in.V_DC_plus,
			COLOUR_DEFAULT, colour_V_DC_minus,
			measurements_in.V_DC_minus,COLOUR_DEFAULT);
	(*p_line_counter)++;
	push_UART2((char*) line_2_str);
}

void draw_UI_line_3(uint8_t* p_line_counter) {

	static char line_3_str[LINE_WIDTH];
	static char colour_V_DC_diff[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_safety.OV_V_DC_diff) {
		/* if the two halves of the bus are too unbalanced */
		strcpy(colour_V_DC_diff, "\x1B[91m"); /* Red*/
	} else {
		strcpy(colour_V_DC_diff, COLOUR_DEFAULT); /* default*/
	}
	sprintf(line_3_str, "V bus diff: %s%2.2f V%s\tV bus total: %2.2f V\n\r",
			colour_V_DC_diff, inverter.V_DC_diff_filtered,
			COLOUR_DEFAULT, inverter.V_DC_total_filtered);
	(*p_line_counter)++;
	push_UART2((char*) line_3_str);
}

void draw_UI_line_4(uint8_t* p_line_counter) {

	static char line_4_str[LINE_WIDTH];
	sprintf(line_4_str, "I AC RMS: %2.2f A\tV AC RMS: %2.2f V\n\r",
			inverter.I_AC_RMS,
			inverter.V_AC_RMS);
	(*p_line_counter)++;
	push_UART2((char*) line_4_str);
}

void draw_UI_line_5(uint8_t* p_line_counter) {

	static char line_5_str[LINE_WIDTH];
	static char power_flow_dir_str[ESCAPE_SEQUENCE_LENGTH];
	if (inverter.P_AC_RMS > 0) {
		/* If power is positive */
		strcpy(power_flow_dir_str, "->"); /* DC towards AC */
	} else {
		/* Else power is negative */
		strcpy(power_flow_dir_str, "<-"); /* AC towards DC */
	}
	sprintf(line_5_str, "P AC RMS: %2.2f W\tPower flow: DC%sAC\n\r",
			inverter.P_AC_RMS, power_flow_dir_str);
	(*p_line_counter)++;
	push_UART2((char*) line_5_str);
}

void draw_UI_line_6(uint8_t* p_line_counter) {

	static char line_6_str[LINE_WIDTH];
	static char xformer_temp_color_str[ESCAPE_SEQUENCE_LENGTH];
	static char inverter_temp_color_str[ESCAPE_SEQUENCE_LENGTH];

	switch(page_number){
		case 1:
			if (inverter_safety.OT_Transformer) {
				/* If transformer is over temperature */
				strcpy(xformer_temp_color_str, "\x1B[91m"); /* Red */
			} else if (inverter_safety.HT_Transformer) {
				/* Else if the transformer has a high temperature*/
				strcpy(xformer_temp_color_str, "\x1B[93m"); /* Yellow */
			} else {
				/* Else the temperature is OK*/
				strcpy(xformer_temp_color_str, COLOUR_DEFAULT); /* default*/
			}

			if (inverter_safety.OT_Inverter) {
				/* If transformer is over temperature */
				strcpy(inverter_temp_color_str, "\x1B[91m"); /* Red */
			} else if (inverter_safety.HT_Inverter) {
				/* Else if the transformer has a high temperature*/
				strcpy(inverter_temp_color_str, "\x1B[93m"); /* Yellow */
			} else {
				/* Else the temperature is OK*/
				strcpy(inverter_temp_color_str, COLOUR_DEFAULT); /* default*/
			}

			sprintf(line_6_str,
					"Xformer temp: %s%2.1fC%s\tInverter temp: %s%2.1fC%s \n\r",
					xformer_temp_color_str, measurements_in.T_transformer,
					COLOUR_DEFAULT, inverter_temp_color_str,
					measurements_in.T_inverter, COLOUR_DEFAULT);
			break;

		case 2:
			sprintf(line_6_str,"Avg. v Mid: %d\n\r",measurement_avgs.v_Mid);
//
			break;
	}

	(*p_line_counter)++;
	push_UART2((char*) line_6_str);
}

void draw_UI_line_7(uint8_t* p_line_counter) {

	static char line_7_str[LINE_WIDTH];

	contactor_state_t AC_contactor_state;
	contactor_state_t DC_contactor_state;


	switch(page_number){

		case 1:
			//

			get_contactor_states(&AC_contactor_state, &DC_contactor_state);

			sprintf(line_7_str, "AC Contactor:");
			if (AC_contactor_state == CONTACTOR_CLOSED) {
				strcat(line_7_str, " Closed");
			} else if (AC_contactor_state == CONTACTOR_PRECHARGE) {
				strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
				strcat(line_7_str, COLOUR_DEFAULT);
			} else if (AC_contactor_state == CONTACTOR_OPEN) {
				strcat(line_7_str, " Open");
			} else if (AC_contactor_state == CONTACTOR_DWELL) {
					strcat(line_7_str, "\x1B[93mDwell"); /*Yellow Dwell*/
					strcat(line_7_str, COLOUR_DEFAULT);
			}

			strcat(line_7_str, "\tDC Contactor:");
			if (DC_contactor_state == CONTACTOR_CLOSED) {
				strcat(line_7_str, " Closed");
			} else if (DC_contactor_state == CONTACTOR_PRECHARGE) {
				strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
				strcat(line_7_str, COLOUR_DEFAULT);
			} else if (DC_contactor_state == CONTACTOR_OPEN) {
				strcat(line_7_str, " Open");
			} else if (DC_contactor_state == CONTACTOR_DWELL) {
					strcat(line_7_str, "\x1B[93mDwell"); /*Yellow Dwell*/
					strcat(line_7_str, COLOUR_DEFAULT);
			}

			strcat(line_7_str, "\n\r");
			break;

		case 2:
			sprintf(line_7_str,"Error code:%x\t\tOper. State:%d\n\r",UI_get_faults_code(),inverter_safety.operating_state);
			break;

	}

	(*p_line_counter)++;
	push_UART2((char*) line_7_str);
}

void draw_UI_line_8(uint8_t* p_line_counter) {

	static char line_8_str[LINE_WIDTH];

	switch(page_number){

		case 1:

			sprintf(line_8_str, "PLL sync: ");
			if (inverter_safety.PLL_sync) {
				strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
				strcat(line_8_str, COLOUR_DEFAULT);
			} else {
				strcat(line_8_str, "\x1B[93mNO"); /* Yellow NO*/
				strcat(line_8_str, COLOUR_DEFAULT);
			}
			strcat(line_8_str, "\t\ti sync: ");
			if (inverter_safety.i_sync) {
				strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
				strcat(line_8_str, COLOUR_DEFAULT);
			} else {
				strcat(line_8_str, "\x1B[91mNO"); /* Red NO*/
				strcat(line_8_str, COLOUR_DEFAULT);
			}
			strcat(line_8_str, "\n\r");
			break;

		case 2:

			sprintf(line_8_str,"i PV avg: %d\t\ti PV offs: %d\n\r",measurement_avgs.i_PV, measurement_offsets.i_PV);
			break;
	}

	(*p_line_counter)++;
	push_UART2((char*) line_8_str);
}

void draw_UI_line_9(uint8_t* p_line_counter) {

	static char line_9_str[LINE_WIDTH];

	switch(page_number){

		case 1:
			sprintf(line_9_str,"PLL freq: %2.2f Hz\n\r",PLL.freq_Hz);
			break;

		case 2:
			sprintf(line_9_str,"v AC avg: %d\t\tv AC offs: %d\n\r",measurement_avgs.v_AC, measurement_offsets.v_AC);
			break;

	}


	(*p_line_counter)++;

	push_UART2((char*) line_9_str);
}


void draw_UI_line_separator(uint8_t* p_line_counter) {

	static char line_separator_str[LINE_WIDTH];
	sprintf(line_separator_str,"****************************************\n\r");
	(*p_line_counter)++;
	push_UART2((char*) line_separator_str);
}

void draw_UI_line_A(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel) {

	static char line_A_str[LINE_WIDTH];
	char colour_on_off_str[ESCAPE_SEQUENCE_LENGTH];
	char on_off_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_v_setpoint_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 0 && col_sel == 0){
		strcpy(colour_on_off_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_on_off_str, COLOUR_NOT_SELECTED);
	}

	if(inverter_setpoints.inverter_active){
		strcpy(on_off_str, "ON");
	}else{
		strcpy(on_off_str, "OFF");
	}

	/* Column 1 */
	if(row_sel == 0 && col_sel == 1){
		strcpy(colour_v_setpoint_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_v_setpoint_str, COLOUR_NOT_SELECTED);
	}


	sprintf(line_A_str,"Inverter: %s%s%s\t\tV DC set: %s%2.2f V%s\n\r",colour_on_off_str,on_off_str,COLOUR_DEFAULT,
			colour_v_setpoint_str,inverter_setpoints.V_DC_total_setpoint,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_A_str);

}

void draw_UI_line_B(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel){

	static char line_B_str[LINE_WIDTH];
	char colour_left_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_V_diff_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 1 && col_sel == 0){
		strcpy(colour_left_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_left_str, COLOUR_NOT_SELECTED);
	}

	/* Column 1 */
	if(row_sel == 1 && col_sel == 1){
		strcpy(colour_V_diff_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_V_diff_str, COLOUR_NOT_SELECTED);
	}

	/*sprintf(line_B_str,"UV2: %s%2.2f%s\t\tV DC diff set: %s%2.2f V%s\n\r",colour_other_str,inverter_setpoints.precharge_threshold,COLOUR_DEFAULT,
			colour_V_diff_str,inverter_setpoints.V_DC_diff_setpoint,COLOUR_DEFAULT);*/

	static char d_ff_zero_str[LINE_WIDTH];

	if(g_d_ff_zero_state){
		strcpy(d_ff_zero_str,"ON");
	}else{
		strcpy(d_ff_zero_str,"OFF");
	}
	sprintf(line_B_str,"d_FF zero: %s%s%s\t\td_FF Offset: %s%i%s\n\r",colour_left_str,d_ff_zero_str,COLOUR_DEFAULT,
	colour_V_diff_str,measurement_offsets.d_FF,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_B_str);

}

void draw_UI_line_C(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel){

	static char line_C_str[LINE_WIDTH];
	char colour_left_value_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_right_value_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 2 && col_sel == 0){
		strcpy(colour_left_value_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_left_value_str, COLOUR_NOT_SELECTED);
	}

	/* Column 1 */
	if(row_sel == 2 && col_sel == 1){
		strcpy(colour_right_value_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_right_value_str, COLOUR_NOT_SELECTED);
	}

	sprintf(line_C_str,"Latch Set Pin: %s%d%s\tComp Reset Pin: %s%d%s\n\r",colour_left_value_str,PA2,COLOUR_DEFAULT,
			colour_right_value_str,PA5,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_C_str);

}

void draw_UI_line_D(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel){

	static char line_D_str[LINE_WIDTH];

	char colour_left_value_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_right_value_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 3 && col_sel == 0){
		strcpy(colour_left_value_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_left_value_str, COLOUR_NOT_SELECTED);
	}

	/* Column 1 */
	if(row_sel == 3 && col_sel == 1){
		strcpy(colour_right_value_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_right_value_str, COLOUR_NOT_SELECTED);
	}

	static char autozero_str[LINE_WIDTH];

	switch(autozero_state){

		case AUTOZERO_STANDBY:
			strcpy(autozero_str,"Standby");
			break;
		case AUTOZERO_WAIT_FOR_CONDITIONS:
			strcpy(autozero_str,"Wait");
			break;
		case AUTOZERO_IN_PROGRESS:
			strcpy(autozero_str,"In Progress");
			break;
		case AUTOZERO_DONE:
			strcpy(autozero_str,"Done");
			break;

	}

	sprintf(line_D_str,"Info page: %s%d%s\t\tAutoZero: %s%s%s\n\r",colour_left_value_str,page_number,COLOUR_DEFAULT,
			colour_right_value_str,autozero_str,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_D_str);

}

void get_contactor_states(contactor_state_t* AC_contactor_state, contactor_state_t*DC_contactor_state){

	*AC_contactor_state = ((PC->PIN)&(BIT2|BIT4))>>2; // Pins for AC contactor are at PC2 and PC4. PC2 is precharge, and PC4 is the relay. Signals are active low.

	*DC_contactor_state = ((PC->PIN)&(BIT3|BIT5))>>3; // Pins for DC contactor are at PC3 and PC5. PC3 is precharge, and PC5 is the relay. Signals are active low.

	// This gives states corresponding to those defined in the contactor_state_t enum.

}

void UI_serialize_code(uint32_t* faults_code, uint8_t bit_number, bool flag){

	if(flag){// If the flag is true
		*faults_code |= (1<<bit_number);// Add a 1
	}


}

uint32_t UI_get_faults_code(void){

	uint32_t faults_code = 0;

	/* Bit 00*/UI_serialize_code(&faults_code, 0, inverter_faults.PLL_sync_fault);
	/* Bit 01*/UI_serialize_code(&faults_code, 1, inverter_faults.i_sync_fault);
	/* Bit 02*/UI_serialize_code(&faults_code, 2, inverter_faults.OV_v_AC_fault);
	/* Bit 03*/UI_serialize_code(&faults_code, 3, inverter_faults.OV_V_DC_plus_fault);
	/* Bit 04*/UI_serialize_code(&faults_code, 4, inverter_faults.OV_V_DC_minus_fault);
	/* Bit 05*/UI_serialize_code(&faults_code, 5, inverter_faults.UV_V_DC_plus_fault);
	/* Bit 06*/UI_serialize_code(&faults_code, 6, inverter_faults.UV_V_DC_minus_fault);
	/* Bit 07*/UI_serialize_code(&faults_code, 7, inverter_faults.UV2_V_DC_plus_fault);
	/* Bit 08*/UI_serialize_code(&faults_code, 8, inverter_faults.UV2_V_DC_minus_fault);
	/* Bit 09*/UI_serialize_code(&faults_code, 9, inverter_faults.OV_V_DC_diff_fault);
	/* Bit 10*/UI_serialize_code(&faults_code, 10, inverter_faults.precharge_timeout_fault);
	/* Bit 11*/UI_serialize_code(&faults_code, 11, inverter_faults.charge_timeout_fault);
	/* Bit 12*/UI_serialize_code(&faults_code, 12, g_Interrupt_real_time_fault);

	return faults_code;
}


