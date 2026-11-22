/*
 * oled.h
 *
 *
 *
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "oled_font.h"

bool Oled_Init(I2C_HandleTypeDef *i2c);

bool Oled_SetFont(const OledFont *font);

bool Oled_WritePage(uint8_t page, const char *text);

#endif /* INC_OLED_H_ */
