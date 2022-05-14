#include "ecg_config.h"
#include <stdbool.h>

//--------------------- Heart Rate ------------------------------
float ecg_compute_heart_rate( uint16_t pk_pos[],uint8_t no_of_peaks)
{
     uint16_t pos_dif = 0.0;
     float t_pos_dif = 0.0;
     float HR_sum = 0.0;
     float HR_avg = 0.0;
     float HR[10] = {0.0};

     for(uint16_t i = 0;i < no_of_peaks-1;i++)
     {
    	pos_dif = pk_pos[i+1] - pk_pos[i];     // difference between two 									//consecutive peak
    	t_pos_dif = ((float)pos_dif)/ECG_ODR;                                               // ECG_output data rate = 100
    	HR[i] = ECG_SECONDS_60/t_pos_dif;          //HR calculation, SECONDS_60 = 60

     }
     for(uint16_t i = 0;i < no_of_peaks-1;i++)
     {
        HR_sum = HR[i] + HR_sum;
     }
       HR_avg = (HR_sum / (no_of_peaks-1));									// To find average of all the individual HRs calculated

return HR_avg;
}


/* bool ecg_peak_detection(uint16_t *ecg_peak_loc, uint8_t *ecg_peak_count)
* \brief        ecg peak detection
* \param[out]   *ecg_peak_loc	  	- peak locations
* \param[out]   *ecg_peak_count  	- number of peaks
*
* \retval      bool, TRUE on successful peak detection
*/

bool ecg_peak_detection(float const filt_output_ECG[600], uint16_t ecg_peak_pos[20], uint8_t *num_peaks)
{
	float ecg_peak_value[20] = {0};
	float temp_filt_max = 0.0;
	uint16_t i = 0;
	uint16_t k = 0;
	uint8_t isMaxValue = 0;
	uint8_t ecg_peak_index = 0;
	uint8_t num_ecg_peaks_grtr_pt5 = 0;
	uint8_t num_ecg_peaks_grtr_pt4 = 0;
	bool status_peak_detection = FALSE;

	MemSet(ecg_peak_value,(float)0x00u,sizeof(ecg_peak_value));
	//MemSet(ecg_peak_pos,(uint16_t)0x00u,sizeof(ecg_peak_pos));
	*num_peaks = 0;

	for (i = ECG_TH_RANGE_RISE_SIDE; i < (ECG_NUM_SAMPLES - ECG_TH_RANGE_FALL_SIDE); i++)
	{
		if ((filt_output_ECG[i] >= ECG_PEAK_DETECT_TH_0pt4) &&
				(filt_output_ECG[i] > filt_output_ECG[i - ECG_TH_RANGE_RISE_SIDE]) &&
				(filt_output_ECG[i] > filt_output_ECG[i + ECG_TH_RANGE_FALL_SIDE]) &&
				((float)(filt_output_ECG[i] - filt_output_ECG[i - ECG_TH_RANGE_RISE_SIDE]) >= ECG_PEAK_DETECT_TH_0pt4) &&
				((float)(filt_output_ECG[i] - filt_output_ECG[i + ECG_TH_RANGE_FALL_SIDE]) >= ECG_PEAK_DETECT_TH_0pt4))
		{
			isMaxValue = 1;

			for (k = (i - ECG_TH_RANGE_RISE_SIDE); k <= (i + ECG_TH_RANGE_FALL_SIDE); k++)
			{
				temp_filt_max = filt_output_ECG[i];

				if (filt_output_ECG[k] > temp_filt_max)
				{
					isMaxValue = 0;
					break;
				}
			}

			if (isMaxValue)
			{
				ecg_peak_pos[ecg_peak_index] = i;
				ecg_peak_value[ecg_peak_index] = temp_filt_max;
				ecg_peak_index++;
				*num_peaks = *num_peaks + 1;
				status_peak_detection = TRUE;
			}
		}

	}

	for(i = 0; (i < *num_peaks) && (status_peak_detection == TRUE) ; i++)
	{
		if(ecg_peak_value[i] >= ECG_PEAK_DETECT_TH_0pt5)
		{
			num_ecg_peaks_grtr_pt5++;
		}
		else if(ecg_peak_value[i] >= ECG_PEAK_DETECT_TH_0pt4)
		{
			num_ecg_peaks_grtr_pt4++;
		}
		else
		{
			status_peak_detection = FALSE;
			break;
		}

	}

	if(status_peak_detection == TRUE)
	{
		if((*num_peaks >= ECG_NUM_MINIMUM_PEAKS) &&
				(num_ecg_peaks_grtr_pt5 >= (*num_peaks/2)) &&
				(num_ecg_peaks_grtr_pt4 <= (*num_peaks - (*num_peaks / 2))))
		{
			status_peak_detection = TRUE;
		}
		else
		{
			status_peak_detection = FALSE;
		}
	}
	return (status_peak_detection);
}


