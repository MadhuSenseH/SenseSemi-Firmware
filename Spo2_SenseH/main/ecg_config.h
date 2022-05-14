
#ifndef SOURCE_ECG_CONFIG_H_
#define SOURCE_ECG_CONFIG_H_

#include "API_utility.h"



#define ECG_ODR              			100														//Output Data Rate is 100
#define ECG_SECONDS_60                 		60														// 60 sec per minute
#define ECG_NUM_PKS_EXPEC				3														// at least number of peaks req for HR measurement
#define ECG_MIN_HR_MEASURE				30														// beats/min
#define ECG_MAX_HR_MEASURE				300
#define ECG_NUM_SEC_DATA_CAPTURE 		((ECG_NUM_PKS_EXPEC * ECG_SECONDS_60)/ECG_MIN_HR_MEASURE)	// find out num sec data capture to meet expected peaks
#define ECG_NUM_SAMPLES					(ECG_ODR * ECG_NUM_SEC_DATA_CAPTURE)						// calculates number of samples

#define ECG_TH_RANGE_RISE_SIDE			3
#define ECG_TH_RANGE_FALL_SIDE			3
#define ECG_MAX_CAPTURE_ITERATIONS		(ECG_SECONDS_60/ECG_NUM_SEC_DATA_CAPTURE)
#define ECG_PEAK_DETECT_TH_0pt4			(float)0.4
#define ECG_PEAK_DETECT_TH_0pt5			(float)0.5
#define ECG_NUM_MINIMUM_PEAKS			ECG_NUM_PKS_EXPEC
#define BP_MAX_NUM_ECG_PEAKS			(uint8_t)5

#define ECG_WAIT_TIME		            5


bool ecg_peak_detection(float const filt_output_ECG[600], uint16_t ecg_peak_pos[20], uint8_t *num_peaks);
float ecg_compute_heart_rate( uint16_t pk_pos[],uint8_t no_of_peaks);

#endif /* SOURCE_ECG_CONFIG_H_ */
