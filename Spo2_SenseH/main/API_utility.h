

#ifndef API_API_UTILITY_H_
#define API_API_UTILITY_H_

#include "stdbool.h"
#include "stdlib.h"
#include <stdint.h>
#define DATA_BUFFER3_LENGTH			28881

/*-----------------------------------MACROS ------------------------------------------------------------------*/
#define TRUE				1
#define FALSE				0
//#define NULL				0

#define PIN_HIGH			1
#define PIN_LOW				0
/*----------------------------------- ENUM ------------------------------------------------------------------*/

typedef enum{
	ENTER =	0,
	NEXT,
	BACK,
	TOTAL_NUM_BTN
}NAVIGATE_BUTTON_TYPE;

typedef enum
{
	PATIENT_REGISTRATION = 0,
	PATIENT_DEREGISTRATION = 1,
	BP_CALIBRATION_FACTORS = 2,
	BP_STD_VALUES = 4,
}PID_REG_SUB_CMD ;
/*----------------------------------- STRUCTURE --------------------------------------------------------*/

typedef struct __attribute__((__packed__))
{
	uint8_t 		sub_command;
	uint8_t 		new_pid[10];
	uint32_t    	sbp_std_val;
	uint32_t    	dbp_std_val;
	float    		sbp_multiplier_val;
	float    		dbp_multiplier_val;
	bool 			valid_bp_test;
}PATIENT_ID_INFO;

PATIENT_ID_INFO bp_calibration;
/*-----------------------------------GLOBAL FUNCTIONS --------------------------------------------------------*/

uint8_t lead_1_status ;

uint8_t BT_flash_buffer[DATA_BUFFER3_LENGTH];

char* StrCat(char* destination, const char* source);
void IntergerToString(char str[], uint32_t num);
void FloatToString(float value,char* dst_str);
void MemSet(void *buffer, uint8_t value, uint32_t size);
void MemCpy(void *dest, void *src, uint32_t size);
uint16_t Get_strlen(const char* source);
uint32_t Length_padding_multiple_of_four(uint32_t len);
void Hex_to_Float(uint8_t hex_val[], float* float_val);

#endif /* API_API_UTILITY_H_ */
