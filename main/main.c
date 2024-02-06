/**
 * @file 		main.c
 * @copyright 	Copyright (c) 2022
 * @license
 * @version 	2.0.0
 * @date 		2022-09-28
 * @author 		Y Pham
 *
 * @brief 		Project (Tena version 2)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "main.h"
/* Private defines ---------------------------------------------------- */
#define FW_VERSION "v0.2.1.0"

#define __CALIBRATION_ON_
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
spi_device_handle_t spi; /*!< Handle for a device on a SPI bus */
/* Private variables -------------------------------------------------- */
static bool g_mode_calibrate = true;
static int g_calibrate_count = 0;
// static int internal_counter = 0;
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void app_main()
{
	spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	lsm.spi_read = lsm6dso_read;
	lsm.spi_write = lsm6dso_write;
	printf("FW Version: %s\n", FW_VERSION);
	printf("\nBootup complete\n\n");
	gpio_init();
	printf("GPIO initialized\n");
	lsm6dso_init(&lsm);
	printf("LSM6DSO initialized\n");
	batt_init();
	printf("Battery initialized\n");
	// bl_spp_init();
	// printf("Bluetooth SPP initialized\n");
	lsm6dso_toggle_fifo(&lsm, 1);
	lsm6dso_toggle_sensor(&lsm, 1);
	xTaskCreate(sensor_task, "timer_evt_task", 2048, NULL, 7, NULL);
	xTaskCreate(led_task, "timer_evt_task", 2048, NULL, 5, NULL);
}
static void sensor_task()
{
	while (1)
	{
		uint16_t fifocount = lsm6dso_read_fifo(&lsm);
		if (fifocount > 0)
		{
#ifdef __CALIBRATION_ON_
			if (g_mode_calibrate)
			{
				for (int i = 0; i < fifocount / 2; i += 1)
				{
					lsm.acc_x_off += lsm.fifo_acc[i * 3 + 0];
					lsm.acc_y_off += lsm.fifo_acc[i * 3 + 1];
					lsm.acc_z_off += lsm.fifo_acc[i * 3 + 2];

					lsm.gyro_x_off += lsm.fifo_gyro[i * 3 + 0];
					lsm.gyro_y_off += lsm.fifo_gyro[i * 3 + 1];
					lsm.gyro_z_off += lsm.fifo_gyro[i * 3 + 2];
					g_calibrate_count++;
				}
				if (g_calibrate_count >= 300)
				{
					lsm.acc_x_off /= g_calibrate_count;
					lsm.acc_y_off /= g_calibrate_count;
					lsm.acc_z_off /= g_calibrate_count;

					lsm.gyro_x_off /= g_calibrate_count;
					lsm.gyro_y_off /= g_calibrate_count;
					lsm.gyro_z_off /= g_calibrate_count;

					g_mode_calibrate = false;
					g_calibrate_count = 0;
				}
			}
			else
			{
				if (fifocount > SAMPLES_MAX)
					fifocount = SAMPLES_MAX;
				for (int i = 0; i < fifocount / 2; i += 1)
				{
					printf(", %f, %f, %f, %f, %f, %f\n",
						   lsm.fifo_acc[i * 3 + 0] - lsm.acc_x_off,
						   lsm.fifo_acc[i * 3 + 1] - lsm.acc_y_off,
						   lsm.fifo_acc[i * 3 + 2] - lsm.acc_z_off,
						   lsm.fifo_gyro[i * 3 + 0] - lsm.gyro_x_off,
						   lsm.fifo_gyro[i * 3 + 1] - lsm.gyro_y_off,
						   lsm.fifo_gyro[i * 3 + 2] - lsm.gyro_z_off);
					strcat(message_buffer, measurement_buffer);
				}
			}
#else
			if (fifocount > SAMPLES_MAX)
				fifocount = SAMPLES_MAX;
			for (int i = 0; i < fifocount / 2; i += 1)
			{
				printf(", %f, %f, %f, %f, %f, %f\n",
					   lsm.fifo_acc[i * 3 + 0] - lsm.acc_x_off,
					   lsm.fifo_acc[i * 3 + 1] - lsm.acc_y_off,
					   lsm.fifo_acc[i * 3 + 2] - lsm.acc_z_off,
					   lsm.fifo_gyro[i * 3 + 0] - lsm.gyro_x_off,
					   lsm.fifo_gyro[i * 3 + 1] - lsm.gyro_y_off,
					   lsm.fifo_gyro[i * 3 + 2] - lsm.gyro_z_off);
				strcat(message_buffer, measurement_buffer);
			}
#endif
		}
		vTaskDelay(pdMS_TO_TICKS(333));
	}
}

static void led_task()
{
	while (1)
	{
		// Read voltage level
		uint8_t vol_level = batt_level();
		// Read static charge
		batt_status_charging_t status;
		batt_status(&status);
		if (status == BATT_STATUS_CHARGING)
		{
			gpio_set_level(LED_GPIO, 1);
		}
		else
		{
			if (vol_level <= 25)
			{
				// Blink led in 0.1s
				gpio_set_level(LED_GPIO, 1);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, 0);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, 1);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, 0);
			}
			else
			{
				gpio_set_level(LED_GPIO, 0);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
void bl_spp_init(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_bluedroid_init()) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_bluedroid_enable()) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
#if (CONFIG_BT_SSP_ENABLED == true)
	/* Set default parameters for Secure Simple Pairing */
	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif
	/*
	 * Set default parameters for Legacy Pairing
	 * Use variable pin, input pin code when pairing
	 */
	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	esp_bt_pin_code_t pin_code;
	esp_bt_gap_set_pin(pin_type, 0, pin_code);
}
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	switch (event)
	{
	case ESP_SPP_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
		esp_bt_dev_set_device_name(DEVICE_NAME);
		esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		// ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
		// 		 param->close.handle, param->close.async);
		lsm6dso_toggle_sensor(&lsm, 0);
		lsm6dso_toggle_fifo(&lsm, 0);
		break;
	case ESP_SPP_START_EVT:
		if (param->start.status == ESP_SPP_SUCCESS)
		{
			// ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
			// 		 param->start.scn);
		}
		else
		{
			ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
		}
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
		esp_log_buffer_char("", param->data_ind.data, param->data_ind.len);
		break;
	case ESP_SPP_CONG_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
		break;
	case ESP_SPP_WRITE_EVT:
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
		g_handler = param->srv_open.handle;
		lsm6dso_toggle_sensor(&lsm, 1);
		g_mode_calibrate = true;
		lsm6dso_toggle_fifo(&lsm, 1);
		break;
	default:
		break;
	}
}
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_BT_GAP_AUTH_CMPL_EVT:
	{
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
			esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
		}
		else
		{
			ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
		}
		break;
	}
	case ESP_BT_GAP_PIN_REQ_EVT:
	{
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
		if (param->pin_req.min_16_digit)
		{
			ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
			esp_bt_pin_code_t pin_code = {0};
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
		}
		else
		{
			ESP_LOGI(SPP_TAG, "Input pin code: 1234");
			esp_bt_pin_code_t pin_code;
			pin_code[0] = '1';
			pin_code[1] = '2';
			pin_code[2] = '3';
			pin_code[3] = '4';
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
		}
		break;
	}
#if (CONFIG_BT_SSP_ENABLED == true)
	case ESP_BT_GAP_CFM_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %lu", param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
		break;
	case ESP_BT_GAP_KEY_NOTIF_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%lu", param->key_notif.passkey);
		break;
	case ESP_BT_GAP_KEY_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
		break;
#endif
	default:
	{
		ESP_LOGI(SPP_TAG, "event: %d", event);
		break;
	}
	}
	return;
}
void gpio_init()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << LED_GPIO);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	gpio_set_level(LED_GPIO, 1);
	vTaskDelay(pdMS_TO_TICKS(1000));
	gpio_set_level(LED_GPIO, 0);
}
/* End of file -------------------------------------------------------- */