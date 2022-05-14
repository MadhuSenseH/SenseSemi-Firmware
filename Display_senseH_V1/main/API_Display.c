#include "API_Display.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "freertos/task.h"
#include "protofonts.h"
#include "icons.h"


// DT018ATFT LCD Controller Commands
#define NOP                     0x00
#define SOFT_RESET              0x01
#define GET_RED_CHANNEL         0x06
#define GET_GREEN_CHANNEL       0x07
#define GET_BLUE_CHANNEL        0x08
#define GET_PIXEL_FORMAT        0x0C
#define GET_POWER_MODE          0x0A
#define GET_ADDRESS_MODE        0x0B
#define GET_DISPLAY_MODE        0x0D
#define GET_SIGNAL_MODE         0x0E
#define GET_DIAGNOSTIC_RESULT   0x0F
#define ENTER_SLEEP_MODE        0x10
#define EXIT_SLEEP_MODE   READ_ID1      0x11
#define ENTER_PARTIAL_MODE      0x12
#define ENTER_NORMAL_MODE       0x13
#define EXIT_INVERT_MODE        0x20
#define ENTER_INVERT_MODE       0x21
#define SET_GAMMA_CURVE         0x26
#define SET_DISPLAY_OFF         0x28
#define SET_DISPLAY_ON          0x29
#define SET_COLUMN_ADDRESS      0x2A
#define SET_PAGE_ADDRESS        0x2B
#define WRITE_MEMORY_START      0x2C
#define WRITE_LUT               0x2D
#define READ_MEMORY_START       0x2E
#define SET_PARTIAL_AREA        0x30
#define SET_SCROLL_AREA         0x33
#define SET_TEAR_OFF            0x34
#define SET_TEAR_ON             0x35
#define SET_ADDRESS_MODE        0x36
#define SET_SCROLL_START        0X37
#define EXIT_IDLE_MODE          0x38
#define ENTER_IDLE_MODE         0x39
#define SET_PIXEL_FORMAT        0x3A
#define WRITE_MEMORY_CONTINUE   0x3C
#define READ_MEMORY_CONTINUE    0x3E
#define SET_TEAR_SCANLINE       0x44
#define GET_SCANLINE            0x45
#define READ_ID1                0xDA
#define READ_ID2                0xDB
#define READ_ID3                0xDC
#define FRAME_RATE_CONTROL1     0xB1
#define FRAME_RATE_CONTROL2     0xB2
#define FRAME_RATE_CONTROL3     0xB3
#define DISPLAY_INVERSION       0xB4
#define SOURCE_DRIVER_DIRECTION 0xB7
#define GATE_DRIVER_DIRECTION   0xB8
#define POWER_CONTROL1          0xC0
#define POWER_CONTROL2          0xC1
#define POWER_CONTROL3          0xC2
#define POWER_CONTROL4          0xC3
#define POWER_CONTROL5          0xC4
#define VCOM_CONTROL1           0xC5
#define VCOM_CONTROL2           0xC6
#define VCOM_OFFSET_CONTROL     0xC7
#define WRITE_ID4_VALUE         0xD3
#define NV_MEMORY_FUNCTION1     0xD7
#define NV_MEMORY_FUNCTION2     0xDE
#define POSITIVE_GAMMA_CORRECT  0xE0
#define NEGATIVE_GAMMA_CORRECT  0xE1
#define GAM_R_SEL               0xF2

#define DISP_MAX_ROWS     159  // 0 to 159
#define DISP_MAX_COLS     127  // 0 to 127
#define DISP_FRAME_LEN    6    // 6 Indicates 9 - bits transmission
#define DISP_TOTAL_PIXELS (128*160)

uint8_t api_disp_display_char (const char *string, const uint8_t *Font, uint16_t FontColor, uint16_t BGColor, uint8_t x, uint8_t y);


spi_device_handle_t disp_spi;

void disp_gpio_config()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (PIN_NUM_DC) | (DISP_CS_PORT) | (DISP_RESET) ;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void disp_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    //gpio_set_level(PIN_NUM_DC, dc);
}



/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */

/*
void api_disp_write_com(spi_device_handle_t spi, const uint8_t cmd,int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    gpio_set_level(PIN_NUM_DC, 0);

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}*/

void api_disp_write_com(uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t t;
	gpio_set_level(PIN_NUM_DC, 0);

	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=8;                     //Command is 8 bits
	t.tx_buffer=&cmd;               //The data is the cmd itself
	t.user=(void*)0;                //D/C needs to be set to 0
	ret=spi_device_transmit(disp_spi, &t);  //Transmit!
	assert(ret==ESP_OK);
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
/*void api_disp_write_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    gpio_set_level(PIN_NUM_DC, 1);

    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    //ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    ret=spi_device_transmit(spi, &t);
    assert(ret==ESP_OK);            //Should have had no issues.
}*/

void api_disp_write_data_1byte(spi_device_handle_t spi, const uint8_t data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    gpio_set_level(PIN_NUM_DC, 1);

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=&data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void api_disp_write_data(uint8_t data)
{

    esp_err_t ret;
    spi_transaction_t t;
    gpio_set_level(PIN_NUM_DC, 1);
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=&data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(disp_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

}

void api_disp_tx_double(uint16_t data)
{
	   esp_err_t ret;
	    spi_transaction_t t;
	    gpio_set_level(PIN_NUM_DC, 1);
	    memset(&t, 0, sizeof(t));       //Zero out the transaction
	    t.length=8;                 //Len is in bytes, transaction length is in bits.
	    t.tx_buffer=&data;               //Data
	    t.user=(void*)1;                //D/C needs to be set to 1
	    ret=spi_device_transmit(&disp_spi, &t);  //Transmit!
	    assert(ret==ESP_OK);
}


bool API_Display_spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num=DISP_MISO,
        .mosi_io_num=DISP_MOSI,
        .sclk_io_num=DISP_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=128*160
    };

    spi_device_interface_config_t devcfg={

        .clock_speed_hz=8*1000*1000,           //Clock out at 10 MHz

        .mode=0,                                //SPI mode 0
        .spics_io_num=DISP_CS_PORT,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=disp_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };



    //Initialize non-SPI GPIOs
//
  gpio_pad_select_gpio(PIN_NUM_DC);
   gpio_pad_select_gpio(DISP_RESET);
    gpio_pad_select_gpio(DISP_CS_PORT);


  gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
   gpio_set_direction(DISP_RESET, GPIO_MODE_OUTPUT);
  gpio_set_direction(DISP_CS_PORT, GPIO_MODE_OUTPUT);

  //gpio_reset_pin(DISP_RESET);



    //Reset the display
   gpio_set_level(DISP_RESET, 0);
       vTaskDelay(100 / portTICK_RATE_MS);
       gpio_set_level(DISP_RESET, 1);
       vTaskDelay(100 / portTICK_RATE_MS);



    //Initialize the SPI bus
    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 0);
   // ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &disp_spi);
    //ESP_ERROR_CHECK(ret);
    //Initialize the LCD

    return true;
}

void API_Display_setup(void)
{
	int orientation = LCD_ORIENTATION0;

	gpio_set_level(DISP_CS_PORT,0);//cs=0
		//api_disp_reset();
	api_disp_write_com(SET_ADDRESS_MODE);
	api_disp_write_data(POWER_CONTROL1);

	api_disp_write_com(SET_PIXEL_FORMAT);
	api_disp_write_data(0x05);

	api_disp_write_com(POWER_CONTROL1); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	api_disp_write_data(GET_BLUE_CHANNEL);
	api_disp_write_data(NOP);

	api_disp_write_com(POWER_CONTROL2); //Set BT[2:0] for AVDD & VCL & VGH & VGL
	api_disp_write_data(0x03);

	api_disp_write_com(POWER_CONTROL3);
	api_disp_write_data(0x05);

	api_disp_write_com(VCOM_CONTROL1); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	api_disp_write_data(0x43);
	api_disp_write_data(0x43);

	api_disp_write_com(VCOM_OFFSET_CONTROL);
	api_disp_write_data(VCOM_CONTROL1);

	api_disp_write_com(FRAME_RATE_CONTROL1);
	api_disp_write_data(GET_BLUE_CHANNEL);
	api_disp_write_data(0x14);


	api_disp_write_com(0xEC); //Set pumping clock frequency
	api_disp_write_data(GET_PIXEL_FORMAT);

	api_disp_write_com(GAM_R_SEL); //Enable Gamma bit
	api_disp_write_data(SOFT_RESET);


	api_disp_write_com(POSITIVE_GAMMA_CORRECT);
	api_disp_write_data(READ_MEMORY_CONTINUE); //p63
	api_disp_write_data(0x1C); //p62
	api_disp_write_data(0x29); //p61
	api_disp_write_data(0x24); //p59
	api_disp_write_data(0x1D); //p57
	api_disp_write_data(0x09); //p50
	api_disp_write_data(0x50); //p43
	api_disp_write_data(0xC8); //p27/36
	api_disp_write_data(0x42); //p20
	api_disp_write_data(0x19); //p13
	api_disp_write_data(0x1C); //p6
	api_disp_write_data(0x0F); //p4
	api_disp_write_data(0x0E); //p2
	api_disp_write_data(0x05); //p1
	api_disp_write_data(NOP); //p0
	api_disp_write_com(0xE1);
	api_disp_write_data(SOFT_RESET); //p63
	api_disp_write_data(0x06); //p62
	api_disp_write_data(0x19); //p61
	api_disp_write_data(0x0B); //p59
	api_disp_write_data(0x12); //p57
	api_disp_write_data(0x10); //p50
	api_disp_write_data(0x27); //p43
	api_disp_write_data(0x58); //p27/36
	api_disp_write_data(SET_PIXEL_FORMAT); //p20
	api_disp_write_data(GET_BLUE_CHANNEL); //p13
	api_disp_write_data(0x1A); //p6
	api_disp_write_data(0x1E); //p4
	api_disp_write_data(0x27); //p2
	api_disp_write_data(SET_PIXEL_FORMAT); //p1
	api_disp_write_data(0x3F); //p0
	api_disp_write_com(0x11); //Exit Sleep

	for(int i=0;i<300;i++)
	{
		for(int j=0;j<200;j++);
	}

	api_disp_write_com(0x29); // Display On

	api_disp_write_com(0x2A); //Set Column Address
	api_disp_write_data(NOP);
	api_disp_write_data(NOP);
	api_disp_write_data(NOP);
	api_disp_write_data(0x7F);

	api_disp_write_com(0x2B); //Set Page Address
	api_disp_write_data(NOP);
	api_disp_write_data(NOP);
	api_disp_write_data(NOP);
	api_disp_write_data(0x9F);
	api_disp_write_com(SET_ADDRESS_MODE);//For setting the display ram memory pointer to start position
	api_disp_write_data(orientation);
	api_disp_write_com(0x2c);
	gpio_set_level(DISP_CS_PORT,1);//cs=0

}



void API_DISP_Clear_Full_Screen(uint16_t color)
{
	int i;
	//uint8_t data_buff[DISP_TOTAL_PIXELS];
	gpio_set_level(DISP_CS_PORT,0);//cs=0
	vTaskDelay(100 / portTICK_RATE_MS);

	api_disp_write_com(SET_COLUMN_ADDRESS); // Column address setting
api_disp_write_data_1byte(disp_spi,0x00,1);         // start address 0
	api_disp_write_data_1byte(disp_spi,0x00,1);         // start address 0
	api_disp_write_data_1byte(disp_spi,0x00,1);         // end address 0
	api_disp_write_data_1byte(disp_spi,0x7f,1);         // end address 127

	api_disp_write_com(SET_PAGE_ADDRESS);  //Row address setting
	api_disp_write_data_1byte(disp_spi,0x00,1);         // start address 0
	api_disp_write_data_1byte(disp_spi,0x00,1);         // start address 0
	api_disp_write_data_1byte(disp_spi,0x00,1);         // end address 0
	api_disp_write_data_1byte(disp_spi,0xA0,1);           // end address 157
	api_disp_write_com(WRITE_MEMORY_START); //command for storing the pixel data in the display_RAM
	printf("\n\n\n");
	vTaskDelay(100 / portTICK_RATE_MS);

	for(i = 0; i < DISP_TOTAL_PIXELS; i++)//Transmit all 128*160 pixel data ,per pixel will have 16bit
	{
		api_disp_write_data_1byte(disp_spi,color>>8,1);
		api_disp_write_data_1byte(disp_spi,color,1);
		//api_disp_write_data(disp_spi,data_buff,DISP_TOTAL_PIXELS);
		//printf("%d,",i);
	}

	gpio_set_level(DISP_CS_PORT,1);
}



uint8_t api_disp_display_char (const char *string, const uint8_t *Font, uint16_t FontColor, uint16_t BGColor, uint8_t x, uint8_t y)
{
	uint8_t count;
	uint8_t xTemp = x;
	uint8_t *tempFontptr;
	uint16_t numOfPixels;
	char *charSeq = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890:% .-/`<>!?";//for 19x10 font -fixed : DO NOT CHANGE
	uint16_t total_pixels = font19x10[0] * font19x10[1];
	gpio_set_level(DISP_CS_PORT,0);

	while (*string!='\0')
	{
		//identify char location
		for (count=0; charSeq[count]!=*string;count++)
		if (count>TOTAL_ASCII_CHARACTERS) return 0;//character doesn't exist


		tempFontptr=(uint8_t *)font19x10+(19*11*count) + 2;
		// Horizontal Address Start Position
		api_disp_write_com(SET_COLUMN_ADDRESS);
		api_disp_write_data(0x00);
		api_disp_write_data(x);
		api_disp_write_data(0x00);
		api_disp_write_data(x+10);
		// Vertical Address end Position
		api_disp_write_com(SET_PAGE_ADDRESS);
		api_disp_write_data(0x00);
		api_disp_write_data(y);
		api_disp_write_data(0x00);
		api_disp_write_data(y+19);

		api_disp_write_com(0x2C);

		for(numOfPixels = 0; numOfPixels < total_pixels ; numOfPixels++)
		{
			if (*tempFontptr==0x00){
				api_disp_write_data(FontColor>>8);
				api_disp_write_data(FontColor);
			}
			else
			{
				api_disp_write_data(BGColor>>8);
				api_disp_write_data(BGColor);
			}
			tempFontptr++;
		}
		gpio_set_level(DISP_CS_PORT,1);
		string++;
		x=x+10;
		if(x>118)
		{
			y=y+19;
			x=xTemp;
		}
		if (y>150)
		return 0;
	}
return 1;
}



 static void api_disp_display_icon(const uint8_t *ImgPointer, uint8_t left_offset, uint8_t top_offset, uint16_t IconColor, uint16_t BGColor){


	uint8_t maxC = 0x00;
	uint8_t maxR = 0x00;
	uint16_t numOfPixels = 0x00;

	gpio_set_level(DISP_CS_PORT,0);

	maxR=*ImgPointer;ImgPointer++;
	maxC=*ImgPointer; ImgPointer++;   //define maximum Row and column for icon

	api_disp_write_com(SET_COLUMN_ADDRESS); // Horizontal Address Start Position
	api_disp_write_data(0x00);
	api_disp_write_data(left_offset);
	api_disp_write_data(0x00);
	api_disp_write_data(left_offset+maxC-1);

	api_disp_write_com(SET_PAGE_ADDRESS); // Vertical Address end Position
	api_disp_write_data(0x00);
	api_disp_write_data(top_offset);
	api_disp_write_data(0x00);
	api_disp_write_data(top_offset+maxR);

	api_disp_write_com(0x2C);

	for(numOfPixels = 0; numOfPixels < maxR*maxC ; numOfPixels++)
	{
		if (*ImgPointer==0x0000){
			api_disp_write_data(IconColor>>8);
			api_disp_write_data(IconColor);
		}
		else{
			api_disp_write_data(BGColor>>8);
			api_disp_write_data(BGColor);
		}
		ImgPointer++;
	}
	gpio_set_level(DISP_CS_PORT,1);
}

 void api_icon_test(void){
 api_disp_display_icon(SPO2Icon1,20,50,BLUE,WHITE);
 }
