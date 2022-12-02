/*
 * XPT2046_touch.c
 *
 *  Created on: Sep 26, 2021
 *      Author: EngJosiah
 */


#include <stdio.h>
#include <stdlib.h>
#include "../ILI9341/XPT2046_touch.h"

#define READ_X 0xD0
#define READ_Y 0x90


bool XPT2046_TouchPressed()
{
    return HAL_GPIO_ReadPin(T_IRQ_GPIO_Port, T_IRQ_Pin) == GPIO_PIN_RESET;
}

bool XPT2046_TouchGetCoordinates(uint16_t* x, uint16_t* y)
{
	static uint32_t timer;
	if(HAL_GetTick() - timer < 200) return false;
	timer = HAL_GetTick();

    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };
    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;

    for(uint8_t i = 0; i < 2; i++)
    {
        if(!XPT2046_TouchPressed())
            break;

        nsamples++;

        HAL_SPI_Transmit(&hspi2, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        HAL_SPI_Transmit(&hspi2, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }

    uint32_t raw_x = (avg_x / nsamples);
    if(raw_x < XPT2046_MIN_RAW_X) raw_x = XPT2046_MIN_RAW_X;
    if(raw_x > XPT2046_MAX_RAW_X) raw_x = XPT2046_MAX_RAW_X;

    uint32_t raw_y = (avg_y / nsamples);
    if(raw_y < XPT2046_MIN_RAW_Y) raw_y = XPT2046_MIN_RAW_Y;
    if(raw_y > XPT2046_MAX_RAW_Y) raw_y = XPT2046_MAX_RAW_Y;

    // Uncomment this line to calibrate touchscreen:
    //printf("raw_x = %d, raw_y = %d\r\n", (int) raw_x, (int) raw_y);

    *x = (raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X);
    *y = (XPT2046_MAX_RAW_Y -raw_y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y);

    //*x = raw_x;
    //*y = raw_y;

    /*switch (rotation) {
	  case 0:
		xraw = 4095 - y;
		yraw = x;
		break;
	  case 1:
		xraw = x;
		yraw = y;
		break;
	  case 2:
		xraw = y;
		yraw = 4095 - x;
		break;
	  default: // 3
		xraw = 4095 - x;
		yraw = 4095 - y;
	}*/

    return true;
}
