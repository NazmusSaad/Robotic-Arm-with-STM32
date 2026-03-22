/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "ssd1306.h"

extern I2C_HandleTypeDef hi2c2;

/* SSD1306 data buffer. This is the buffer you must write to for setting pixel values. */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* The set of registers to write to when initializing the SSD1306 screen. */
static uint8_t SSD1306_Init_Config [] = {0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF, 0x2E};

/* An array you can use for the commands to be sent for scrolling. */
static uint8_t SSD1306_Scroll_Commands [8] = {0};

/* The following functions are provided for you to use, without requiring any modifications */

void SSD1306_UpdateScreen(void) {
	uint8_t m;
	uint8_t packet[3] = {0x00, 0x00, 0x10};

	for (m = 0; m < 8; m++) {
		packet[0] = (0xB0 + m);
		ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, packet, 3);
		ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Use memset to efficiently set the entire SSD1306_Buffer to a single value. */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void SSD1306_Clear (void)
{
	SSD1306_Fill (0);
    SSD1306_UpdateScreen();
}

/* Start of the functions you must complete for this lab. */
void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {

	uint8_t new[256];
	new[0] = reg;

	for (uint8_t i = 0; i < count; i++) {
		new[i+1] = data[i];
	}

	// its handle, address, pointer to start of array to transmit, the size of the data, and the timeout
	HAL_I2C_Master_Transmit(&hi2c2, address, new, count + 1, 10);
}

HAL_StatusTypeDef SSD1306_Init(void) {

	/* Check if the OLED is connected to I2C */
	if (HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK) {
		return HAL_ERROR;
	}

    /* Keep this delay to prevent overflowing the I2C controller */
	HAL_Delay(10);

	/* Init LCD */
	for (int i = 0; i < sizeof(SSD1306_Init_Config); i++) {
	    ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, &SSD1306_Init_Config[i], 1);
	}

	/* Clear screen */
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	/* Update screen */
	SSD1306_UpdateScreen();

	/* Return OK */
	return HAL_OK;
}



HAL_StatusTypeDef SSD1306_SetPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
		return HAL_ERROR;
	}

	uint16_t byteIndex = x + (y / 8) * SSD1306_WIDTH;
	uint8_t bitPosition = y % 8;

    /* Set the pixel at position (x,y) to 'color'. */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[byteIndex] |= (1 << bitPosition);
	} else {
		SSD1306_Buffer[byteIndex] &= ~(1 << bitPosition);
	}

	return HAL_OK;
}



void SSD1306_Scroll(SSD1306_SCROLL_DIR_t direction, uint8_t start_row, uint8_t end_row)
{
    /* TO DO */
	//so pick the correct hexx command (0x26 or 0x27) defined in the header to selecte the direction of scrolling
	if (direction == SSD1306_SCROLL_RIGHT) {
		SSD1306_Scroll_Commands[0] = SSD1306_RIGHT_HORIZONTAL_SCROLL; // 0x26
	} else {
		SSD1306_Scroll_Commands[0] = SSD1306_LEFT_HORIZONTAL_SCROLL;  // 0x27
	}

	//given byte
	SSD1306_Scroll_Commands[1] = 0x00;

	//page to start from
	SSD1306_Scroll_Commands[2] = start_row;

	// scrolling frequency
	SSD1306_Scroll_Commands[3] = 0x00;

	//page to stop at
	SSD1306_Scroll_Commands[4] = end_row;

	//given byte
	SSD1306_Scroll_Commands[5] = 0x00;

	// another given byte
	SSD1306_Scroll_Commands[6] = 0xFF;

	// activate scrolling
	SSD1306_Scroll_Commands[7] = SSD1306_ACTIVATE_SCROLL; // 0x2F

	//transmit commands
	ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, SSD1306_Scroll_Commands, 8);
}

void SSD1306_Stopscroll(void)
{
	/* TO DO */
	uint8_t command = SSD1306_DEACTIVATE_SCROLL; // 0x2E
	ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, &command, 1);
}


void SSD1306_Putc(uint16_t x, uint16_t y, char ch, FontDef_t* Font) {

    /* TODO */
	int index = ch - 32;

	// loop over 18 rows of bitmap
	for (int row = 0; row < 18; row++){
		uint16_t row_bits = Font_11x18.data[index * 18 + row];

		for (int col = 0; col < 11; col++){
			//loop over top 11 bits in the row and check which ones we need to set
			// use a mask and shift to compare each bit with each col
			if (row_bits & (1 << (15 - col))){
				SSD1306_SetPixel(x + col, y + row, SSD1306_COLOR_WHITE);
			}
		}
	}
}

HAL_StatusTypeDef SSD1306_Puts(char* str, FontDef_t* Font) {
	int cur_x = 0;
	int cur_y = 0;
	/* Loop over every character until we see \0. */
	while (*str != '\0') {

        /* TODO */

		if (*str == '\n'){
			cur_y += 18;
			cur_x = 0;
		}
		else{
			SSD1306_Putc(cur_x, cur_y, *str, Font);
			cur_x += 11;
		}

        /* Increase string pointer */
        str++;
	}

	return HAL_OK;
}
