/**
 * @file       batt.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-09-27
 * @author     Y Pham
 *
 * @brief       Driver for battery
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BATT_H_
#define _BATT_H_
/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
/* Public defines ----------------------------------------------------- */
#define DEFAULT_VREF        (1100)
#define NO_OF_SAMPLES       (64) // Multisampling
#define STT_CHGR GPIO_NUM_17
#define BATT_MAX            (5000)
#define BATT_MIN            (2000)
#define BATT_VOLTAGE_3700   (3700)
#define BATT_VOLTAGE_3800   (3800)
#define BATT_VOLTAGE_3900   (3900)
#define BATT_VOLTAGE_4000   (4000)
#define BATT_VOLTAGE_4050   (4050)
#define BATT_VOLTAGE_4200   (4200)
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
       BATT_STATUS_CHARGING = 0,
       BATT_STATUS_DISCHARGING
} batt_status_charging_t;
/* Public macros ------------------------------------------------------ */
/* Function definitions ----------------------------------------------- */
/**
 * @brief Initialize battery
 *
 * @return  read voltage initial
 */
void batt_init();

/**
 * @brief Read voltage level of battery
 *
 * @return voltage level in percent
 */
uint8_t batt_level();

/**
 * @brief Read status charging battery
 *
 * @param[out] status status charging
 */
void batt_status(batt_status_charging_t *status);
#endif /* _BATT_H_ */
/* End of file -------------------------------------------------------- */