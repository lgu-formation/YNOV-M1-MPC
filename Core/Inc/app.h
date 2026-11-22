/*
 * Header app.h
 *
 * App_Init and App_Run interfaces
 * Public interface for FSM
 *
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/* HAL interface */
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Application States */
typedef enum {
	APP_INIT = 0,
	APP_CALIBRATION,
	APP_RUN,
	APP_PAUSE,
	APP_ERROR
} AppState;

/* Init function of the application */
void App_Init(I2C_HandleTypeDef *i2c);

/* Run function of the application */
void App_Run(void);

/* Interrupt callback from GPIO_EXTI */
void App_OnExti(uint16_t pin);

/* Getter for current AppState */
AppState App_GetState(void);

#endif /* INC_APP_H_ */
