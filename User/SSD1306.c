/*
 * SSD1306.c
 *
 *  Created on: Jun 1, 2024
 *      Author: �굧�럹
 */


#include "SSD1306.h"
#include "debug.h"
#include "font.h"


#define SSD1306_I2C_ADDR 0x3C

uint8_t ssd1306_buffer[128]={0};
uint8_t buffer[1024];

void I2C_PORTInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1| GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_InitStructure.I2C_ClockSpeed = 400000;      // 100000
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

void I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void SSD1306_datas(uint8_t* data, uint16_t size) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, SSD1306_I2C_ADDR << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, 0x40);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    for(uint16_t i = 0; i < size; i++) {
        I2C_SendData(I2C1, data[i]);
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
    I2C_GenerateSTOP(I2C1, ENABLE);
}


void SSD1306_Command(uint8_t cmd) {
    I2C_Write(0x78, 0x00, cmd);
}

void SSD1306_Data(uint8_t data) {
    I2C_Write(0x78, 0x40, data);
}


void OLED_Init(void)
{
    SSD1306_Command(0xAE);        //   Set display OFF
    SSD1306_Command(0xD4);        //   Set Display Clock Divide Ratio / OSC Frequency
    SSD1306_Command(0x80);        // the suggested ratio 0x80 Display Clock Divide Ratio / OSC Frequency
    SSD1306_Command(0xA8);        //  Set Multiplex Ratio
    SSD1306_Command(64 - 1);      //Multiplex Ratio for 128x64 (64-1)
    SSD1306_Command(0xD3);        // Set Display Offset
    SSD1306_Command(0x0);          // Display Offset no offset
    SSD1306_Command(0x40);        // line #0 Set Display Start Line
    SSD1306_Command(0x8D);        // Set Charge Pump
    SSD1306_Command(0x14);        // Charge Pump (0x10 External, 0x14 Internal DC/DC) using internal VCC
    SSD1306_Command(SSD1306_MEMORYMODE);                    // 0x20
    SSD1306_Command(0x00);                                  // 0x00 horizontal addressing mode
    SSD1306_Command(0xA1);         //Set Segment Re-Map
    SSD1306_Command(0xC8);          // Set Com Output Scan Direction
    SSD1306_Command(0xDA);         // Set COM Hardware Configuration
    SSD1306_Command(0x12);          // COM Hardware Configuration
    SSD1306_Command(0x81);          // Set Contrast
    SSD1306_Command(0xCF);          // Contrast
    SSD1306_Command(0xD9);          // Set Pre-Charge Period
    SSD1306_Command(0xF1);          // Set Pre-Charge Period (0x22 External, 0xF1 Internal)
    SSD1306_Command(0xDB);          // Set VCOMH Deselect Level
    SSD1306_Command(0x40);          // VCOMH Deselect Level
    SSD1306_Command(0xA4);          // Set all pixels OFF
    SSD1306_Command(0xA6);          // Set display not inverted
    SSD1306_Command(0xAF);          // Set display On
}

void OLED_SetCursor(vu8 x, vu8 y)
{
    SSD1306_Command(0xB0 + y);
    SSD1306_Command(((x & 0xF0) >> 4) | 0x10);
    SSD1306_Command((x & 0x0F) | 0x01);
}


void SSD1306_CLearDisplay()
{
    SSD1306_Command(0x20);
    SSD1306_Command(0x00); // Horizontal addressing mode
    for (uint16_t i = 0; i < 1024; i++) {
        SSD1306_Data(0x00);
    }
}



// �뵿�� �꽕�젙 �븿�닔
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= 128 || y >= 64) {
        return;
    }
    if (color) {
        buffer[x + (y / 8) * 128] |= (1 << (y % 8));
    } else {
        buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
    }
}

// 踰꾪띁瑜� �뵒�뒪�뵆�젅�씠�뿉 �쟾�넚
void SSD1306_Display(void)
{
    SSD1306_Command(0x21); // Set column address
    SSD1306_Command(0);    // Start column: 0
    SSD1306_Command(127);  // End column: 127
    SSD1306_Command(0x22); // Set page address
    SSD1306_Command(0);    // Start page: 0
    SSD1306_Command(7);    // End page: 7

    for (uint16_t i = 0; i < 1024; i++) {
        SSD1306_Data(buffer[i]);
    }
}

// �꽑 洹몃━湲� �븿�닔 (Bresenham's line algorithm)
void SSD1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        SSD1306_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}



void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    for (uint8_t i = 0; i < w; i++) {
        for (uint8_t j = 0; j < h; j++) {
            SSD1306_DrawPixel(x + i, y + j, color);
        }
    }
}


void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color)
{
    if (ch < 32 || ch > 127) {
        ch = 32; // 怨듬갚�쑝濡� ��泥�
    }

    for (int8_t i = 0; i < 8; i++ ) { // 5x7 �룿�듃�쓽 �꼫鍮�
        uint8_t line = FONTS8[ch - 32][i]; // 湲��옄 �뜲�씠�꽣 媛��졇�삤湲�

        for (int8_t j = 0; j < 7; j++, line >>= 1) { // 湲��옄 �뜲�씠�꽣�쓽 媛� 鍮꾪듃�뿉 ���빐
            if (line & 1) {
                if (size == 1) {
                    SSD1306_DrawPixel(x + i, y + j, color);
                } else {
                    SSD1306_FillRect(x + (i * size), y + (j * size), size, size, color);
                }
            } else {
                if (size == 1) {
                    SSD1306_DrawPixel(x + i, y + j, !color);
                } else {
                    SSD1306_FillRect(x + (i * size), y + (j * size), size, size, !color);
                }
            }
        }
    }
}


void SSD1306_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t size, uint8_t color)
{
    while (*str) {
        if (x + 6 * size >= 128) { // 以꾨컮轅�
            x = 0;
            y += 8 * size;
            if (y + 7 * size >= 64) { // �뵒�뒪�뵆�젅�씠 �쁺�뿭�쓣 踰쀬뼱�굹硫� 醫낅즺
                break;
            }
        }
        SSD1306_DrawChar(x, y, *str, size, color);
        x += 6 * size; // �떎�쓬 臾몄옄 �쐞移�
        str++;
    }
}







