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
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static esp_adc_cal_characteristics_t *adc_chars;
/* Private function prototypes ---------------------------------------- */
/**
 * @brief Read voltage
 * @return voltage in mV
 */
static uint32_t batt_read();
/* Function definitions ----------------------------------------------- */
void batt_init()
{
    /* Configure ADC */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    /* Characterize ADC */
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    /* Configure Charge status */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << STT_CHGR);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    /* Read battery */
    uint32_t voltage = batt_read();
    printf("Voltage: %d\n", voltage);
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
    *status = gpio_get_level(STT_CHGR);
}
/* Private Function definitions --------------------------------------- */
static uint32_t batt_read()
{
    /*  Read sample ADC1 */
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_reading /= NO_OF_SAMPLES;
    /* Convert adc_reading to voltage in mV */
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    uint32_t voltage_batt = (voltage * (100 + 51)) / 100;
    if (voltage_batt > BATT_MAX || voltage_batt < BATT_MIN)
    {
        return 0;
    }
    return voltage_batt;
}
/* End of file -------------------------------------------------------- */