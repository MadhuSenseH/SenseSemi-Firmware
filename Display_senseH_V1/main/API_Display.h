#ifndef API_DISPLAY_H
#define API_DISPLAY_H
#include <stdbool.h>
#include <stdint.h>


#define LCD_ORIENTATION0    0
#define LCD_ORIENTATION1    96
#define LCD_ORIENTATION2    160
#define LCD_ORIENTATION3    192



#define PIN_NUM_DC   13
#define DISP_CS_PORT 4
#define DISP_MOSI    26//12
#define DISP_MISO   27//2
#define DISP_CLK    19//14
#define DISP_RESET   2



#define BLACK           0x0000      //  0, 0, 0
#define WHITE           0xFFFF      // 31,31,31
#define BLUE           0x001F       // 31, 0, 0
#define GREEN           0x07C0      //  0,31, 0
#define NAVY            0x000F      //  0, 0,15
#define DARKGREEN       0x03C0      //  0,15, 0
#define DARKCYON        0x03EF      //   0, 128,128
#define MAROON          0x7800      // 15, 0, 0
#define PURPLE          0x780F      // 128,   0, 128
#define OLIVE           0x7BE0      // 128, 128,   0
#define LIGHTGREY       0xC618      // 192, 192, 192
#define DARKGREY        0x7BEF      // 128, 128, 128
#define RED             0xF800      // 0, 0,31
#define CYON            0x07DF      //  0,31,31
#define YELLOW          0xF81F       // 31, 0,31
#define MAGENTA         0xFFC0      // 31,31, 0
#define ORANGE          0xFD20      // 255, 165,   0
#define GREENYELLOW     0xAFE5      // 173, 255,  47
#define LIGHTBLUE       0x04FF

#define TOTAL_ASCII_CHARACTERS  95

bool API_Display_spi_init(void);
void API_Display_setup(void);

void API_DISP_Clear_Full_Screen(uint16_t color);
uint8_t api_disp_display_char (const char *string, const uint8_t *Font, uint16_t FontColor, uint16_t BGColor, uint8_t x, uint8_t y);
void api_icon_test(void);











#endif
