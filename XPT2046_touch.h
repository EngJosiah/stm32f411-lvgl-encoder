/*
 * XPT2046_touch.h
 *
 *  Created on: Sep 26, 2021
 *      Author: EngJosiah
 */

#ifndef ILI9341_XPT2046_TOUCH_H_
#define ILI9341_XPT2046_TOUCH_H_

#include "main.h"
#include <stdbool.h>

/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 2.5 Mbit speed, better ~650 Kbit to be save.
extern SPI_HandleTypeDef hspi2;
#define XPT2046_IRQ_Pin       T_IRQ_Pin
#define XPT2046_IRQ_GPIO_Port T_IRQ_GPIO_Port

// change depending on screen orientation
#define XPT2046_SCALE_X 240
#define XPT2046_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define XPT2046_MIN_RAW_X 3400
#define XPT2046_MAX_RAW_X 29000
#define XPT2046_MIN_RAW_Y 3300
#define XPT2046_MAX_RAW_Y 30000

// call before initializing any SPI devices
bool XPT2046_TouchPressed();
bool XPT2046_TouchGetCoordinates(uint16_t* x, uint16_t* y);

#endif /* ILI9341_XPT2046_TOUCH_H_ */
