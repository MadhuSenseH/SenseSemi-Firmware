/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "ppg_config.h"
#include "quick_test.h"
#include "API_SPO2.h"

#define MAX30102_ADDR  0xAE
#define CONFIG_I2C_MASTER_PORT_NUM 1

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = '\0';



/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, int size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR  | READ_BIT), ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t ppg_reg_read(i2c_port_t i2c_num, uint8_t reg_addr,uint8_t *data_rd)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_ADDR , ACK_CHECK_EN);

    i2c_master_write_byte(cmd, reg_addr , ACK_CHECK_EN);

    printf("red_addr=%x\n",reg_addr);

    i2c_master_write_byte(cmd, MAX30102_ADDR | 1, ACK_CHECK_EN);

    *data_rd=0;
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);

    printf("red_data=%x\n",*data_rd);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, int size,bool enable_read)
{
/*    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    if(enable_read)
    {
        i2c_master_write_byte(cmd, MAX30102_ADDR, ACK_CHECK_EN);

    }

    else
    {
        i2c_master_write_byte(cmd, (MAX30102_ADDR| WRITE_BIT), ACK_CHECK_EN);

    }
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;




    */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
           i2c_master_write_byte(cmd, *data_wr << 1 | WRITE_BIT, ACK_CHECK_EN);
           i2c_master_write_byte(cmd, *(data_wr+1), ACK_CHECK_EN);
           i2c_master_start(cmd);

       i2c_master_write_byte(cmd, *data_wr << 1 | READ_BIT, ACK_CHECK_EN);

       int data=0;
       i2c_master_read_byte(cmd, &data, NACK_VAL);
       i2c_master_stop(cmd);
       esp_err_t ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
       i2c_cmd_link_delete(cmd);


       printf("\n data=%X",data);

       return ret;
}



static esp_err_t api_spo2_read_reg(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg_addr, uint8_t *reg_data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
           i2c_master_write_byte(cmd, slave_addr | WRITE_BIT, ACK_CHECK_EN);
           i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
           i2c_master_start(cmd);

       i2c_master_write_byte(cmd, slave_addr | READ_BIT, ACK_CHECK_EN);

       i2c_master_read_byte(cmd,reg_data, NACK_VAL);
       i2c_master_stop(cmd);
       esp_err_t ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
       i2c_cmd_link_delete(cmd);

       return ret;
}


static esp_err_t api_spo2_write_reg(i2c_port_t i2c_num, uint8_t slave_addr,uint8_t reg_add, uint8_t reg_data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_data, NACK_VAL);

	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);


	return ret;
}
static esp_err_t test(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (MAX30102_ADDR| WRITE_BIT), ACK_CHECK_EN);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_test_task(void *arg)
{

    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);

    int cnt = 0;

    memset(data,0xFE,DATA_LENGTH);

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        xSemaphoreTake(print_mux, portMAX_DELAY);

        // i2c_master_write_slave(I2C_MASTER_NUM, data, 512);
        /*if (d_size == 0) {
            ESP_LOGW(TAG, "i2c slave tx buffer full");
            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);
        } else {
            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_TEST_LENGTH);
        }*/


        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
        //---------------------------------------------------
       /* int size;
        for (i = 0; i < DATA_LENGTH; i++) {
            data_wr[i] = i + 10;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        //we need to fill the slave buffer so that master can read later
        ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_TEST_LENGTH);
        if (ret == ESP_OK) {
            size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        }
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER WRITE TO SLAVE\n", task_idx);
            printf("*******************\n");
            printf("----TASK[%d] Master write ----\n", task_idx);
            disp_buf(data_wr, RW_TEST_LENGTH);
            printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
            disp_buf(data, size);
        } else {
            ESP_LOGW(TAG, "TASK[%d] %s: Master write slave error, IO not connected....\n",
                     task_idx, esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);*/
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}
/*

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(API_BP_PPG_Register_Init, "i2c_test_task_0", 1024 * 2, (void *)0, 1, NULL);
}
*/


esp_err_t API_BP_PPG_Register_Init(void)
{
		esp_err_t reg_init_status = 0xFF;


		//reg_init_status = api_bp_ppg_reset();
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_LED1_PA,0x20);

		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_INTR_ENABLE_1,INTR_ENABLE_1);			// Interrupt enabled  when new data is ready
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_INTR_ENABLE_2,0x02);           // Disable interrupt for die temperature
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_FIFO_WR_PTR,FIFO_WR_PTR);  				// FIFO Write Pointer points to the location where the MAX30102 writes the next sample
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_OVF_COUNTER,OVF_COUNTER);  				// OVF_COUNTER counts the number of samples lost. It saturates at 0xF.
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_FIFO_RD_PTR,FIFO_RD_PTR);  				// FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_FIFO_CONFIG,FIFO_CONFIG_SpO2);  				// 1 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_MODE_CONFIG,0x03);   			// Heart Rate mode, Red LED only
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_SPO2_CONFIG,0x36);  			// SPO2_ADC range = 4096nA, SPO2 sample rate - 1000sps, LED pulseWidth - (215uS)
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_LED1_PA,0xFF);   					// Choose value for approx. 7.4mA for LED1
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_LED2_PA,0xFF);   					// Choose value for approx. 7.4mA for LED2
		reg_init_status  |= api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_PILOT_PA,0xFF);   					// Choose value for approx. 25.4mA for Pilot LED

		return reg_init_status;

}

void API_PPG_CHIP_TEST(void)
{

	//BP_ERROR_STATUS error_status = BP_INIT_ERROR;
	//	float temperature = 0;
	bool reg_init_status = false;

	uint8_t write_data[2] = {0xAE,REG_LED1_PA};
	uint8_t read_data=0;

	write_data[0] = write_data[0] >> 1;
	static bool flag = false;


	//Quick_Test();

 	if(!flag)
 	{
	i2c_master_init();
  api_spo2_write_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,REG_LED1_PA,0x20);
  API_BP_PPG_Register_Init();
	flag = true;
 	}

 //i2c_master_write_slave(I2C_MASTER_NUM,write_data, 1,false);

   api_spo2_read_reg(I2C_MASTER_NUM,PPG_DEVICE_ADDRESS,0xFF,&read_data);

   printf("read_value = %X\n",read_data);

//    ppg_reg_read(I2C_MASTER_NUM,write_data,read_data);
/*

//while(1){}
	while(1){

   // xSemaphoreTake(print_mux, portMAX_DELAY);

    ppg_reg_read(I2C_MASTER_NUM,REG_INTR_STATUS_2,read_data);


	//i2c_master_write_slave(I2C_MASTER_NUM, write_data, 2,false);

	//i2c_master_write_slave(I2C_MASTER_NUM, write_data, 2,true);

	//i2c_master_read_slave(I2C_MASTER_NUM, read_data, 1);

    xSemaphoreGive(print_mux);

 	printf("%X",read_data[0]);



    printf("\n");
	}
*/


/*
		reg_init_status |= api_bp_ppg_write_reg(REG_IMAX30102_ADDRNTR_ENABLE_2,INTR_ENABLE_2);           // Disable interrupt for die temperature
		reg_init_status |= api_bp_ppg_write_reg(REG_FIFO_WR_PTR,FIFO_WR_PTR);  				// FIFO Write Pointer points to the location where the MAX30102 writes the next sample
		reg_init_status |= api_bp_ppg_write_reg(REG_OVF_COUNTER,OVF_COUNTER);  				// OVF_COUNTER counts the number of samples lost. It saturates at 0xF.
		reg_init_status |= api_bp_ppg_write_reg(REG_FIFO_RD_PTR,FIFO_RD_PTR);  				// FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO
		reg_init_status |= api_bp_ppg_write_reg(REG_FIFO_CONFIG,FIFO_CONFIG_SpO2);  				// 1 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
		reg_init_status |= api_bp_ppg_write_reg(REG_MODE_CONFIG,MODE_CONFIG_BP);   			// Heart Rate mode, Red LED only
		reg_init_status |= api_bp_ppg_write_reg(REG_SPO2_CONFIG,SPO2_CONFIG_400ODR);  			// SPO2_ADC range = 4096nA, SPO2 sample rate - 800sps, LED pulseWidth - (118uS)
		reg_init_status |= api_bp_ppg_write_reg(REG_LED1_PA,LED1_PA);   					// Choose value for approx. 7.4mA for LED1
		reg_init_status |= api_bp_ppg_write_reg(REG_LED2_PA,LED2_PA);   					// Choose value for approx. 7.4mA for LED2
		reg_init_status |=api_bp_ppg_write_reg(REG_PILOT_PA,PILOT_PA);   					// Choose value for approx. 25.4mA for Pilot LED
*/

	//}

	if(reg_init_status == true)
	{
		printf("No error");
	}
	//return error_status;


}
