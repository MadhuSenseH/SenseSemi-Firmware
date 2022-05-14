#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "API_Display.h"
#include "sdkconfig.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}
SemaphoreHandle_t print_mux = NULL;

void delay()
{
	/*for(int i=0;i<500;i++)
	{
		for(int j=0;j<50;j++){}
	}*/
}

void app_main(void)
{
    //nvs_flash_init();
    //tcpip_adapter_init();
    /*ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }*/


	  printf("SPI Setup..........\n");

		/* gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT_OD);

	  while(1)
	 	 {
	 	 gpio_set_level(GPIO_NUM_15, 0);
	 	 for(int i=0;i<5000;i++){}
	 	 gpio_set_level(GPIO_NUM_15, 1);
	 	 for(int i=0;i<5000;i++){}

	 	 }*/
	API_Display_spi_init();
	API_Display_setup();
	API_DISP_Clear_Full_Screen(WHITE);
	 api_disp_display_char ("  SPO2 TEST ", 0 ,BLUE, WHITE, 2, 2);

	 api_icon_test();
	 while(1);
	while(1){
	API_DISP_Clear_Full_Screen(YELLOW);
	API_DISP_Clear_Full_Screen(BLUE);
	API_DISP_Clear_Full_Screen(GREEN);
	API_DISP_Clear_Full_Screen(BLACK);
	}
	//vTaskDelay(5000 / portTICK_RATE_MS);
	//API_DISP_Clear_Full_Screen(YELLOW);

	while(1){}
	while(1)
	{

		API_DISP_Clear_Full_Screen(WHITE);
		delay();
		API_DISP_Clear_Full_Screen(BLUE);
		delay();
		API_DISP_Clear_Full_Screen(RED);
		delay();

		API_DISP_Clear_Full_Screen(GREEN);
		delay();

	  printf(".");

	}

	  printf("SPI Setup done\n");
	while(1){}

}

