/*
 * SSD1306.h
 *
 *  Created on: Jun 1, 2024
 *      Author: °øÀå
 */

#ifndef USER_SSD1306_H_
#define USER_SSD1306_H_

#ifndef SSD1306_H
#define SSD1306_H

#include "debug.h"


// I2C address for the SSD1306
//#define SSD1306_I2C_ADDRESS 0x3C

// SSD1306 commands
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_SEGREMAP 0xA1
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22

// Screen dimensions
#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64

// Initialization function
extern void I2C_PORTInit(void);
extern void OLED_Init(void);

// Function to send a command to the SSD1306
void ssd1306_command(vu8 cmd);

// Function to send data to the SSD1306
void ssd1306_data(vu8 data);

// Function to update the screen

extern void OLED_SetCursor(vu8 x, vu8 y) ;


extern void SSD1306_CLearDisplay();
extern void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color);
extern void SSD1306_Display(void);
extern void SSD1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
extern void SSD1306_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t size, uint8_t color);

#endif // SSD1306_H


#endif /* USER_SSD1306_H_ */
