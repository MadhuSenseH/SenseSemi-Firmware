#include "API_utility.h"
#include "ecg_config.h"

#include <math.h>

static double dv1_100[7] = { 0.236886761822703, -0.0, -0.710660285468109,
								-0.0, 0.710660285468109, -0.0,-0.236886761822703 };			// numerator

static double dv3_100[7] = { 1.0,-2.3280703658159174,1.6133375869559861,-0.5507609478544464,
								0.54658760619782754,-0.2366971729435301,-0.043892402907766553 };	// denominator

static double b_a_100[6] = { -0.23688676182239832, -0.23688676182310764,0.47377352364549286,
								0.473773523645325, -0.2368867618226175,-0.23688676182268964 };

static double dv1_200[7] = { 0.045617789533832623, -0.0,-0.13685336860149788, -0.0,
								0.13685336860149788, -0.0, -0.045617789533832623};

static double dv3_200[7] = { 1.0, -4.1540113037465849, 7.2029642951693837,-6.80728494155853,
								3.7709727399491575, -1.1610869625748115,0.14845860595407684 };

static double b_a_200[6] = { -0.045617789539959833, -0.045617789514507338,0.091235579042856479,
								0.091235579084566157, -0.045617789540037215,-0.045617789532922996 };

float y_1[1200 + 36] = {0};
float b_y_1[1200 + 36] = {0};
float dv0_1[7] = {0};
float dv2_1[7] = {0};
float a_1[6] = {0};

static void filtfilt(float raw_data[],float filtered_data[],uint16_t total_num_ofsamples,uint16_t sampling_rate);
static void filt(uint16_t total_num_ofsamples);
//bool ecg_peak_detection(float const filt_output_ECG[600], uint16_t ecg_peak_pos[20], uint8_t *num_peaks);
//float ecg_compute_heart_rate( uint16_t pk_pos[],uint8_t no_of_peaks);



void filter(float raw_data[], float filtered_data[],uint16_t total_num_ofsamples,uint16_t data_rate)
{
	float ex = 0;
	float extemp = 0;
	uint16_t k;

	 //Read the Raw data and apply the IIR BPF
	filtfilt(raw_data,filtered_data,total_num_ofsamples,data_rate);

	//b_abs(output_ECG, varargin_1);
	//replace b_abs function with code
	ex = abs(filtered_data[0]);

	for (k = 0; k < (total_num_ofsamples-1); k++)
	{
		extemp = abs(filtered_data[k + 1]);
		if (ex < extemp)
		{
			ex = extemp;
		}
	}

	for (k = 0; k < total_num_ofsamples; k++)
	{
		filtered_data[k] /= ex;
	}
}


/* Arguments    : const float x_in[600]
*                float y_out[600]
* Return Type  : void
*/
static void filtfilt(float raw_data[],float filtered_data[],uint16_t total_num_ofsamples,uint16_t sampling_rate)
{
	float d0 = 0;
	float d1 = 0;
	float ytmp = 0;
	uint16_t i = 0;
	double dv1[7] = {0};
	double dv3[7] = {0};
 	double b_a[6] = {0};

 	switch (sampling_rate)
	{
		case 100:
				MemCpy(dv1,dv1_100,sizeof(dv1));
				MemCpy(dv3,dv3_100,sizeof(dv3));
				MemCpy(b_a,b_a_100,sizeof(b_a));

				break;
		case 200:
			MemCpy(dv1,dv1_200,sizeof(dv1));
			MemCpy(dv3,dv3_200,sizeof(dv3));
			MemCpy(b_a,b_a_200,sizeof(b_a));

			break;

	}

	d0 = (float)2.0 * raw_data[0];
	d1 = (float)2.0 * raw_data[total_num_ofsamples-1];
	for (i = 0; i < 18; i++)
	{
		y_1[i] = d0 - raw_data[18 - i];
	}

	MemCpy(&y_1[18], &raw_data[0], (total_num_ofsamples * sizeof(float)));

	for (i = 0; i < 18; i++)
	{
		y_1[i + total_num_ofsamples + 18] = d1 - raw_data[(total_num_ofsamples-2) - i];
	}

	for (i = 0; i < 7; i++)
	{
		dv0_1[i] = dv1[i];
		dv2_1[i] = dv3[i];
	}

	for (i = 0; i < 6; i++)
	{
		a_1[i] = b_a[i] * y_1[0];
	}

	MemCpy(&b_y_1[0], &y_1[0], (total_num_ofsamples + 36) * sizeof(float));
	filt(total_num_ofsamples);

	// replace flipud
	for (i = 0; i < ((total_num_ofsamples/2) + 18); i++)
	{
		ytmp = y_1[i];
		y_1[i] = y_1[(total_num_ofsamples+35) - i];
		y_1[(total_num_ofsamples+35) - i] = ytmp;
	}

	for (i = 0; i < 7; i++)
	{
		dv0_1[i] = dv1[i];
		dv2_1[i] = dv3[i];
	}

	for (i = 0; i < 6; i++)
	{
		a_1[i] = b_a[i] * y_1[0];
	}

	MemCpy(&b_y_1[0], &y_1[0], (total_num_ofsamples + 36) * sizeof(float));
	filt(total_num_ofsamples);

	// replace flipud
	for (i = 0; i < ((total_num_ofsamples/2) + 18); i++)
	{
		ytmp = y_1[i];
		y_1[i] = y_1[(total_num_ofsamples+35) - i];
		y_1[(total_num_ofsamples+35) - i] = ytmp;
	}
	MemCpy(&filtered_data[0], &y_1[18], (total_num_ofsamples * sizeof(float)));
}


/* Arguments    : float b[7]
 *                float a_1[7]
 *                const float x[636]
 *                const float zi[6]
 *                float y_1[636]
 * Return Type  : void
*/

static void filt(uint16_t total_num_ofsamples)
{

	float a1;
	uint16_t k;
	uint16_t naxpy;
	uint16_t j;

	a1 = dv2_1[0];
	if ((!(dv2_1[0] == 0.0)) && (dv2_1[0] != 1.0)) {
		for (k = 0; k < 7; k++)
		{
			dv0_1[k] /= a1;
		}


		for (k = 0; k < 6; k++)
		{
			dv2_1[k + 1] /= a1;
		}

		dv2_1[0] = 1.0;
	}

	for (k = 0; k < 6; k++)
	{
		y_1[k] = a_1[k];
	}

	MemSet(&y_1[6], 0, (total_num_ofsamples+ 30) * sizeof(float));
	for (k = 0; k < (total_num_ofsamples+ 36); k++)
	{
		naxpy = (total_num_ofsamples+ 36) - k;
		if (!(naxpy < 7))
		{
			naxpy = 7;
		}

		for (j = 0; j < naxpy; j++)
		{
			y_1[k + j] += b_y_1[k] * dv0_1[j];
		}

		naxpy = (total_num_ofsamples+ 35) - k;
		if (!(naxpy < 6))
		{
			naxpy = 6;
		}

		a1 = -y_1[k];
		for (j = 1; j <= naxpy; j++)
		{
			y_1[k + j] += a1 * dv2_1[j];
		}
	}
}



