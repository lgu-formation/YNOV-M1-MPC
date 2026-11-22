/*
 * app_config.h
 *
 * Application configuration
 *
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#include <stdint.h>

/* Witness LED period */
#define APP_LED_PERIOD_MS		(500U)

/* IMU acquisition period when interruption not used */
#define APP_IMU_PERIOD_MS		(10U)

/* IMU watchdog for interruption monitoring */
#define APP_IMU_WATCHDOG_MS		(250U)

/* IMU calibration duration (2 seconds) */
#define APP_CALIBRATION_SAMPLES	(200U)

/* Authorized IMU failures */
#define APP_MAX_IMU_FAILURES	(3U)

/* OLED page refresh */
#define APP_OLED_PAGE_PERIOD_MS	(20U)

/* Debounce */
#define APP_BUTTON_DEBOUNCE_MS	(40U)

/* Délai ISR */
#define TP_ISR_LONG_WAIT          (0U)
#define TP_ISR_WAIT_MS            (30U)

/* --- USB Device CDC_FS : pour com série USB (télémétrie). --- */
#define TP_USB_CDC_ENABLED        (1U)
#define TP_TELEMETRY_PERIOD_MS    (50U)

#endif /* INC_APP_CONFIG_H_ */
