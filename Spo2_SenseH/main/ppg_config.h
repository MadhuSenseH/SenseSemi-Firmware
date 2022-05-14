#ifndef _H_PPG_CONFIG_H_
#define _H_PPG_CONFIG_H_


/*-------------------------------------MACROS-------------------------------------------------------------------*/
#define PPG_DEVICE_ADDRESS				0xAE
#define PPG_WRITE_ADDRESS				0xAE							//Command to write to maxim
#define PPG_READ_ADDRESS				0xAF							//Command to read from MAXIM
#define PPG_FREQUENCY					400000

#define TIMEOUT							(uint32_t)300000
#define RAW_SAMPLE_MASK					(uint32_t)0x03FFFF



/* Configuration Registers*/
#define REG_INTR_STATUS_1 			(uint8_t)0x00u
#define REG_INTR_STATUS_2 			(uint8_t)0x01u
#define REG_INTR_ENABLE_1 			(uint8_t)0x02u
#define REG_INTR_ENABLE_2 			(uint8_t)0x03u
#define REG_FIFO_WR_PTR 			(uint8_t)0x04u
#define REG_OVF_COUNTER 			(uint8_t)0x05u
#define REG_FIFO_RD_PTR 			(uint8_t)0x06u
#define REG_FIFO_DATA 				(uint8_t)0x07u
#define REG_FIFO_CONFIG 			(uint8_t)0x08u
#define REG_MODE_CONFIG 			(uint8_t)0x09u
#define REG_SPO2_CONFIG 			(uint8_t)0x0Au
#define REG_LED1_PA 				(uint8_t)0x0Cu
#define REG_LED2_PA 				(uint8_t)0x0Du
#define REG_PILOT_PA 				(uint8_t)0x10u


/* Values to Configuration Registers*/
#define RESET						(uint8_t)0x40u
#define INTR_ENABLE_1				(uint8_t)0xC0u					// Interrupt enabled  when new data is ready
#define INTR_ENABLE_2				(uint8_t)0x00u					// Disable interrupt for die temperature
#define FIFO_WR_PTR					(uint8_t)0x00u					// FIFO Write Pointer points to the location where the MAX30102 writes the next sample
#define OVF_COUNTER					(uint8_t)0x00u					// OVF_COUNTER counts the number of samples lost. It saturates at 0xF.
#define FIFO_RD_PTR					(uint8_t)0x00u					// FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO

#define LED1_PA						(uint8_t)0x24u					// Choose value for approx. 7.4mA for LED1
#define LED2_PA						(uint8_t)0x24u					// Choose value for approx. 7.4mA for LED2
#define PILOT_PA					(uint8_t)0x7Fu					// Choose value for approx. 25.4mA for Pilot LED



/* BP Config */
#define MODE_CONFIG_BP				(uint8_t)0x02u					// Heart Rate mode, Red LED only
#define FIFO_CONFIG_BP				(uint8_t)0x0Fu					// 1 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
#define SPO2_CONFIG_BP				(uint8_t)0x29u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 200sps, LED pulseWidth - (118uS)



/* SPO2 Config */

#define MODE_CONFIG_SpO2			(uint8_t)0x03u					// Enable Red LED and IR LED
#define FIFO_CONFIG_SpO2			(uint8_t)0x2Fu					// 2 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
#define SPO2_CONFIG_SpO2			(uint8_t)0x29u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 200sps, LED pulseWidth - (411uS)




//#define FIFO_CONFIG					(uint8_t)0x0Fu					// 1 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17

//800sps
//#define SPO2_CONFIG_BP				(uint8_t)0x31u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 800sps, LED pulseWidth - (118uS)

//500sps
//#define FIFO_CONFIG				(uint8_t)0x2Fu					// 2 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
//#define SPO2_CONFIG				(uint8_t)0x35u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 1000sps, LED pulseWidth - (118uS)



// 100sps
//#define SPO2_CONFIG				(uint8_t)0x25u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 100sps, LED pulseWidth - (118uS)

// 200sps
//#define SPO2_CONFIG					(uint8_t)0x29u					// SPO2_ADC range = 4096nA, SPO2 sample rate - 200sps, LED pulseWidth - (118uS)

// 400sps
#define SPO2_CONFIG_400ODR				(uint8_t)0x2Du					// SPO2_ADC range = 4096nA, SPO2 sample rate - 400sps, LED pulseWidth - (118uS)



#endif
