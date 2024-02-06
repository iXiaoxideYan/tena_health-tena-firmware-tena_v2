/**
 * @file       batt.c
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-09-27
 * @author     Y Pham
 *
 * @brief       Driver for battery
 *
 */
/* Includes ----------------------------------------------------------- */
#include "batt.h"
/* Private defines ---------------------------------------------------- */

#if CONFIG_IDF_TARGET_ESP32
#define     ADC1_CHAN0      ADC_CHANNEL_6
#else
#define     ADC1_CHAN0      ADC_CHANNEL_9 // No defination for 11, so i choose 9 here
#endif

#define     ADC_ATTEN       ADC_ATTEN_DB_11
#define     TAG             "BT_GAP"
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
static int adc_raw[2][10];
static int voltage_array[2][10];
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_chan0_handle = NULL;
bool do_calibration1_chan0 = false;
/* Private variables -------------------------------------------------- */
// static esp_adc_cal_characteristics_t *adc_chars;
static bool adc_calibration_init(adc_unit_t unit, 
                                 adc_channel_t channel, 
                                 adc_atten_t atten, 
                                 adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
/* Private function prototypes ---------------------------------------- */
/**
 * @brief Read voltage
 * @return voltage in mV
 */
static uint32_t batt_read();
/* Function definitions ----------------------------------------------- */
void batt_init()
{
    /* ADC1 Init */
    // adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    /* Configure ADC */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));

    /* ADC1 Calibration Init */
    // adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);

    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    // /* Characterize ADC */
    // adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    // /* Configure Charge status */
    // gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pin_bit_mask = (1ULL << STT_CHGR);
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // gpio_config(&io_conf);
    /* Read battery */
    uint32_t voltage = batt_read();
    printf("Voltage: %ld\n", voltage);
}

uint8_t batt_level()
{
    uint32_t voltage_batt = batt_read();
    if (voltage_batt <= BATT_VOLTAGE_3700)
    {
        return 25;
    }
    else if (voltage_batt > BATT_VOLTAGE_3800 && voltage_batt < BATT_VOLTAGE_3900)
    {
        return 50;
    }
    else if (voltage_batt >= BATT_VOLTAGE_4000 && voltage_batt < BATT_VOLTAGE_4050)
    {
        return 75;
    }
    else if (voltage_batt >= BATT_VOLTAGE_4200)
    {
        return 100;
    }
    else
    {
        if (voltage_batt <= BATT_VOLTAGE_3800)
        {
            return (35 * voltage_batt) / BATT_VOLTAGE_3800;
        }
        else if (voltage_batt <= BATT_VOLTAGE_4000)
        {
            return (70 * voltage_batt) / BATT_VOLTAGE_4000;
        }
        else
        {
            return (90 * voltage_batt) / BATT_VOLTAGE_4200;
        }
    }
}
void batt_status(batt_status_charging_t *status)
{
    // *status = gpio_get_level(STT_CHGR);
}
/* Private Function definitions --------------------------------------- */
static uint32_t batt_read(){
    // /*  Read sample ADC1 */
    // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1, ADC1_CHAN0, ));
    // uint32_t adc_reading = 0;
    uint32_t voltage = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
        if(do_calibration1_chan0){
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage_array[0][0]));
            voltage += voltage_array[0][0];
        }
        // adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if(do_calibration1_chan0){
        adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    voltage /= NO_OF_SAMPLES;
    /* Convert adc_reading to voltage in mV */
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    uint32_t voltage_batt = (voltage * (100 + 51)) / 100;
    if (voltage_batt > BATT_MAX || voltage_batt < BATT_MIN)
    {
        return 0;
    }
    return voltage_batt;
}
/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle){
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    #endif
}

/* End of file -------------------------------------------------------- */