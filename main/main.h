/**
 * @file 		main.h
 * @copyright 	Copyright (c) 2022
 * @license
 * @version 	2.0.0
 * @date 		2022-09-28
 * @author 		Y Pham
 *
 * @brief 		Project (Tena version 2)
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _MAIN_H_
#define _MAIN_H_
/* Includes ----------------------------------------------------------- */
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include "nvs.h"
#include "nvs_flash.h"
#ifndef __FREERTOS__
#define __FREERTOS__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif
#include "esp_system.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"
#include "spi_flash_mmap.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/timer_group_struct.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lsm6dso.h"
#include "batt.h"
#include "bsp.h"
/* Public defines ----------------------------------------------------- */
/* Bluetooth SPP defines */
#define SPP_TAG                 "Bluetooth_IMU"
#define SPP_SERVER_NAME         "SPP_SERVER"
#define DEVICE_NAME             "Bluetooth_IMU_D"
#define SPP_SHOW_DATA           (0)
#define SPP_SHOW_SPEED          (1)
#define SPP_SHOW_MODE SPP_SHOW_SPEED
/* Sample defines */
#define SAMPLES_MAX             (100)
/*  GPIO defines */
#define GPIO_NUM_17             (17)
#define LED_GPIO GPIO_NUM_22
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/**
 * @brief Configuration structure for a SPI bus.
 * 
 */
spi_bus_config_t buscfg =
    {
        .miso_io_num = 5,
        .mosi_io_num = 18,
        .sclk_io_num = 23,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512};
/**
 * @brief Configuration for a SPI slave device that is connected to one of the SPI buses.
 * 
 */
spi_device_interface_config_t devcfg =
    {
        .command_bits = 0,
        .address_bits = 8,
        .mode = 3,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = 5 * 1000 * 1000, /**< 5MHz CLK */ 
        .spics_io_num = 19,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 12,
};
uint32_t g_handler;
lsm6dso_t lsm;
/* Private variables -------------------------------------------------- */
static char measurement_buffer[200*2] = "\0";
static char message_buffer[3000] = "\0";
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
/* Private function prototypes ---------------------------------------- */
static void sensor_task();
static void led_task();
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
/* Function definitions ----------------------------------------------- */
/**
 * @brief Initializes GPIO
 * 
 */
void gpio_init();
/**
 * @brief Initializes bluetooth
 * 
 */
void bl_spp_init(void);
/**
 * @brief Bluetooth GAP callback
 * 
 * @param[in] event BT GAP callback events
 * @param[out] param GAP state callback parameters
 */
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
#endif /* _MAIN_H_ */
/* End of file -------------------------------------------------------- */