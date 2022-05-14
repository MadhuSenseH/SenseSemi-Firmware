#ifndef SOURCE_BPF_H_
#define SOURCE_BPF_H_

#include <stdint.h>


void filter(float raw_data[], float filtered_data[],uint16_t total_num_ofsamples,uint16_t data_rate);

#endif /* SOURCE_BPF_H_ */
