/**
 * @file       bsp.c
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-10-06
 * @author     Y Pham
 *
 * @brief      BSP layer
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp.h"
/* Private defines ---------------------------------------------------- */
#define READ_SPI  (0x80)
#define WRITE_SPI (0x00)
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
lsm6dso_err_t lsm6dso_write(uint8_t register_address, uint8_t value)
{
    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = register_address | WRITE_SPI;
    trans.length = 8;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_data[0] = value;
    CHECK(spi_device_transmit(spi, &trans) == ESP_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_read(uint8_t register_address, uint8_t *value, uint8_t len)
{
    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = register_address | READ_SPI;
    trans.length = 8 * len;
    trans.rxlength = 8 * len;
    trans.tx_buffer = NULL;
    CHECK(spi_device_transmit(spi, &trans) == ESP_OK, LSM6DSO_ERR);
    for (int i = 0; i < len; i++)
    {
        value[i] = trans.rx_data[i];
    }
    return LSM6DSO_OK;
}
/* Private Function definitions --------------------------------------- */
/* End of file -------------------------------------------------------- */
