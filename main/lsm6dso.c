/**
 * @file       lsm6dso.c
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-09-14
 * @author     Y Pham
 *
 * @brief       Driver for lsm6ds0
 *
 */

/* Includes ----------------------------------------------------------- */
#include "lsm6dso.h"
#include <string.h>
/* Private defines ---------------------------------------------------- */
#define PROPERTY_ENABLE     (0x01)
#define PROPERTY_DISENABLE  (0x00)
/* Self test results. */
#define ST_PASS (1U)
#define ST_FAIL (0U)
/* Self test limits. */
#define MIN_ST_LIMIT_G      (0.05f)
#define MAX_ST_LIMIT_G      (1.7f)
#define MIN_ST_LIMIT_DPS    (150.0f)
#define MAX_ST_LIMIT_DPS    (700.0f)
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define CHECK(_A_, _OUT_) \
    do                    \
    {                     \
        if (!(_A_))       \
            return _OUT_; \
    } while (0)
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */
/**
 * @brief Check chip ID
 *
 * @return lsm6dso_err_t
 */
static lsm6dso_err_t lsm6dso_whoami(lsm6dso_t *lsm);

/**
 * @brief  Block data update.[set]
 *
 * @param[in]  val      change the values of bdu in reg CTRL3_C
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_block_data_update_set(lsm6dso_t *lsm, uint8_t vale);

/**
 * @brief  Accelerometer full-scale selection.[set]
 *
 * @param[in]  val      change the values of fs_xl in reg CTRL1_XL
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_acc_full_scale_set(lsm6dso_t *lsm, lsm6dso_fs_acc_t val);
/**
 * @brief  Gyro full-scale selection.[set]
 *
 * @param[in]  val      change the values of fs_xl in reg CTRL1_XL
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_gyro_full_scale_set(lsm6dso_t *lsm, lsm6dso_fs_gyro_t val);
/**
 * @brief  I3C Enable/Disable communication protocol[.set]
 *
 * @param[in]  val      change the values of i3c_disable in reg CTRL9_XL
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_i3c_disable_set(lsm6dso_t *lsm, uint8_t val);

/**
 * @brief  Final State Machine enable.[get]
 *
 * @param[out]  val      union of registers from FSM_ENABLE_A to FSM_ENABLE_B
 * @return lsm6dso_err_t
 */
static lsm6dso_err_t lsm6dso_fsm_enable_get(lsm6dso_t *lsm, uint16_t *val);

/**
 * @brief  Enable access to the embedded functions/sensor
 *         hub configuration registers.[set]
 *
 * @param[in]  val      change the values of reg_access in reg FUNC_CFG_ACCESS
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_embedd_set(lsm6dso_t *lsm, lsm6dso_reg_access_t val);

/**
 * @brief  Finite State Machine ODR configuration.[get]
 *
 * @param[out]  val      Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_fsm_data_rate_get(lsm6dso_t *lsm, lsm6dso_fsm_odr_t *val);

/**
 * @brief  Software reset. Restore the default values in user registers[set]
 *
 * @param[in]  val      change the values of sw_reset in reg CTRL3_C
 * @return lsm6dso_err_t
 *
 */
static lsm6dso_err_t lsm6dso_reset_set(lsm6dso_t *lsm, uint8_t val);

/**
 * @brief self test check
 *
 * @return lsm6dso_err_t
 */
static lsm6dso_err_t lsm6dso_self_test(lsm6dso_t *lsm);

/* Function definitions ----------------------------------------------- */

lsm6dso_err_t lsm6dso_init(lsm6dso_t *lsm)
{
    memset(lsm->fifo_acc, 0, sizeof(lsm->fifo_acc));
    memset(lsm->fifo_gyro, 0, sizeof(lsm->fifo_gyro));
    CHECK(lsm6dso_whoami(lsm) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_reset_set(lsm, PROPERTY_ENABLE) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_i3c_disable_set(lsm, PROPERTY_ENABLE) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_block_data_update_set(lsm, PROPERTY_ENABLE) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_gyro_full_scale_set(lsm, LSM6DSO_500DPS) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_acc_full_scale_set(lsm, LSM6DSO_4G) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_self_test(lsm) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_acc_data_rate_set(lsm, LSM6DSO_XL_ODR_104HZ) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_gyro_data_rate_set(lsm, LSM6DSO_GY_ODR_104HZ) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_fifo_watermark_set(lsm, 10) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_fifo_acc_batch_set(lsm, LSM6DSO_XL_BATCHED_AT_104HZ) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_fifo_gyro_batch_set(lsm, LSM6DSO_GY_BATCHED_AT_104HZ) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_fifo_mode_set(lsm, LSM6DSO_STREAM_MODE) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(lsm6dso_toggle_sensor(lsm, 0) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_acc_batch_set(lsm6dso_t *lsm, lsm6dso_bdr_acc_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_CTRL3, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_FIFO_CTRL3_BDR_XL_MASK);
    reg |= (uint8_t)val;
    CHECK(lsm->spi_write(LSM6DSO_FIFO_CTRL3, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_gyro_batch_set(lsm6dso_t *lsm, lsm6dso_bdr_gyro_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_CTRL3, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_FIFO_CTRL3_BDR_GY_MASK);
    reg |= (uint8_t)val << 4;
    CHECK(lsm->spi_write(LSM6DSO_FIFO_CTRL3, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_mode_set(lsm6dso_t *lsm, lsm6dso_fifo_mode_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_CTRL4, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_FIFO_CTRL4_FIFO_MODE_MASK);
    reg |= val;
    CHECK(lsm->spi_write(LSM6DSO_FIFO_CTRL4, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_watermark_set(lsm6dso_t *lsm, uint16_t val)
{
    uint8_t reg_2;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_CTRL2, &reg_2, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg_2 &= ~(LSM6DSO_FIFO_CTRL2_WTM_MASK);
    reg_2 |= ((0x0100U & val) >> 8);
    CHECK(lsm->spi_write(LSM6DSO_FIFO_CTRL2, reg_2) == LSM6DSO_OK, LSM6DSO_ERR);
    uint8_t reg_1;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_CTRL1, &reg_1, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg_1 = (0x00FF & val);
    CHECK(lsm->spi_write(LSM6DSO_FIFO_CTRL1, reg_1) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_acc_data_rate_set(lsm6dso_t *lsm, lsm6dso_odr_acc_t val)
{
    lsm6dso_odr_acc_t odr_xl = val;
    uint16_t fsm_enable;
    lsm6dso_fsm_odr_t fsm_odr;
    CHECK(lsm6dso_fsm_enable_get(lsm, &fsm_enable) == LSM6DSO_OK, LSM6DSO_ERR);
    if (fsm_enable >= PROPERTY_ENABLE)
    {
        CHECK(lsm6dso_fsm_data_rate_get(lsm, &fsm_odr) == LSM6DSO_OK, LSM6DSO_ERR);
        switch (fsm_odr)
        {
        case LSM6DSO_ODR_FSM_12HZ5:
            if (val == LSM6DSO_XL_ODR_OFF)
            {
                odr_xl = LSM6DSO_XL_ODR_12HZ5;
            }
            else
            {
                odr_xl = val;
            }
            break;
        case LSM6DSO_ODR_FSM_26HZ:
            if (val == LSM6DSO_XL_ODR_OFF)
            {
                odr_xl = LSM6DSO_XL_ODR_26HZ;
            }
            else if (val == LSM6DSO_XL_ODR_12HZ5)
            {
                odr_xl = LSM6DSO_XL_ODR_26HZ;
            }
            else
            {
                odr_xl = val;
            }
            break;
        case LSM6DSO_ODR_FSM_52HZ:
            if (val == LSM6DSO_XL_ODR_OFF)
            {
                odr_xl = LSM6DSO_XL_ODR_52HZ;
            }
            else if (val == LSM6DSO_XL_ODR_12HZ5)
            {
                odr_xl = LSM6DSO_XL_ODR_52HZ;
            }
            else if (val == LSM6DSO_XL_ODR_26HZ)
            {
                odr_xl = LSM6DSO_XL_ODR_52HZ;
            }
            else
            {
                odr_xl = val;
            }
            break;
        case LSM6DSO_ODR_FSM_104HZ:
            if (val == LSM6DSO_XL_ODR_OFF)
            {
                odr_xl = LSM6DSO_XL_ODR_104HZ;
            }
            else if (val == LSM6DSO_XL_ODR_12HZ5)
            {
                odr_xl = LSM6DSO_XL_ODR_104HZ;
            }
            else if (val == LSM6DSO_XL_ODR_26HZ)
            {
                odr_xl = LSM6DSO_XL_ODR_104HZ;
            }
            else if (val == LSM6DSO_XL_ODR_52HZ)
            {
                odr_xl = LSM6DSO_XL_ODR_104HZ;
            }
            else
            {
                odr_xl = val;
            }
            break;
        default:
            odr_xl = val;
            break;
        }
    }
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL1_XL, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL1_XL_ORD_XL_MASK);
    reg |= odr_xl << 4;
    CHECK(lsm->spi_write(LSM6DSO_CTRL1_XL, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_gyro_data_rate_set(lsm6dso_t *lsm, lsm6dso_odr_gyro_t val)
{
    lsm6dso_odr_gyro_t odr_gy = val;
    uint16_t fsm_enable;
    lsm6dso_fsm_odr_t fsm_odr;
    CHECK(lsm6dso_fsm_enable_get(lsm, &fsm_enable) == LSM6DSO_OK, LSM6DSO_ERR);
    if (fsm_enable >= PROPERTY_ENABLE)
    {
        CHECK(lsm6dso_fsm_data_rate_get(lsm, &fsm_odr) == LSM6DSO_OK, LSM6DSO_ERR);
        switch (fsm_odr)
        {
        case LSM6DSO_ODR_FSM_12HZ5:
            if (val == LSM6DSO_GY_ODR_OFF)
            {
                odr_gy = LSM6DSO_GY_ODR_12HZ5;
            }
            else
            {
                odr_gy = val;
            }
            break;
        case LSM6DSO_ODR_FSM_26HZ:
            if (val == LSM6DSO_GY_ODR_OFF)
            {
                odr_gy = LSM6DSO_GY_ODR_26HZ;
            }
            else if (val == LSM6DSO_GY_ODR_12HZ5)
            {
                odr_gy = LSM6DSO_GY_ODR_26HZ;
            }
            else
            {
                odr_gy = val;
            }
            break;
        case LSM6DSO_ODR_FSM_52HZ:
            if (val == LSM6DSO_GY_ODR_OFF)
            {
                odr_gy = LSM6DSO_GY_ODR_52HZ;
            }
            else if (val == LSM6DSO_GY_ODR_12HZ5)
            {
                odr_gy = LSM6DSO_GY_ODR_52HZ;
            }
            else if (val == LSM6DSO_GY_ODR_26HZ)
            {
                odr_gy = LSM6DSO_GY_ODR_52HZ;
            }
            else
            {
                odr_gy = val;
            }
            break;
        case LSM6DSO_ODR_FSM_104HZ:
            if (val == LSM6DSO_GY_ODR_OFF)
            {
                odr_gy = LSM6DSO_GY_ODR_104HZ;
            }
            else if (val == LSM6DSO_GY_ODR_12HZ5)
            {
                odr_gy = LSM6DSO_GY_ODR_104HZ;
            }
            else if (val == LSM6DSO_GY_ODR_26HZ)
            {
                odr_gy = LSM6DSO_GY_ODR_104HZ;
            }
            else if (val == LSM6DSO_GY_ODR_52HZ)
            {
                odr_gy = LSM6DSO_GY_ODR_104HZ;
            }
            else
            {
                odr_gy = val;
            }
            break;
        default:
            odr_gy = val;
            break;
        }
    }
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL2_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL2_G_ODR_G_MASK);
    reg |= odr_gy << 4;
    CHECK(lsm->spi_write(LSM6DSO_CTRL2_G, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_wtm_flag_get(lsm6dso_t *lsm, uint8_t *val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_STATUS2, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    *val = (reg & 0x80) >> 7;
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_data_level_get(lsm6dso_t *lsm, uint16_t *val)
{
    uint8_t reg;
    uint8_t data[2];
    CHECK(lsm->spi_read(LSM6DSO_FIFO_STATUS1, &reg, 2) == LSM6DSO_OK, LSM6DSO_ERR);
    data[0] = reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_STATUS2, &reg, 2) == LSM6DSO_OK, LSM6DSO_ERR);
    data[1] = reg;
    *val = (data[0]) | (data[1] & 0x3) << 8;
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_sensor_tag_get(lsm6dso_t *lsm, lsm6dso_fifo_tag_t *val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_TAG, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    uint8_t tag_sensor;
    tag_sensor = (reg & 0xF8) >> 3;
    switch (tag_sensor)
    {
    case LSM6DSO_GYRO_NC_TAG:
        *val = LSM6DSO_GYRO_NC_TAG;
        break;
    case LSM6DSO_XL_NC_TAG:
        *val = LSM6DSO_XL_NC_TAG;
        break;
    case LSM6DSO_TEMPERATURE_TAG:
        *val = LSM6DSO_TEMPERATURE_TAG;
        break;
    case LSM6DSO_CFG_CHANGE_TAG:
        *val = LSM6DSO_CFG_CHANGE_TAG;
        break;
    case LSM6DSO_XL_NC_T_2_TAG:
        *val = LSM6DSO_XL_NC_T_2_TAG;
        break;
    case LSM6DSO_XL_NC_T_1_TAG:
        *val = LSM6DSO_XL_NC_T_1_TAG;
        break;
    case LSM6DSO_XL_2XC_TAG:
        *val = LSM6DSO_XL_2XC_TAG;
        break;
    case LSM6DSO_XL_3XC_TAG:
        *val = LSM6DSO_XL_3XC_TAG;
        break;
    case LSM6DSO_GYRO_NC_T_2_TAG:
        *val = LSM6DSO_GYRO_NC_T_2_TAG;
        break;
    case LSM6DSO_GYRO_NC_T_1_TAG:
        *val = LSM6DSO_GYRO_NC_T_1_TAG;
        break;
    case LSM6DSO_GYRO_2XC_TAG:
        *val = LSM6DSO_GYRO_2XC_TAG;
        break;
    case LSM6DSO_GYRO_3XC_TAG:
        *val = LSM6DSO_GYRO_3XC_TAG;
        break;
    case LSM6DSO_SENSORHUB_SLAVE0_TAG:
        *val = LSM6DSO_SENSORHUB_SLAVE0_TAG;
        break;
    case LSM6DSO_SENSORHUB_SLAVE1_TAG:
        *val = LSM6DSO_SENSORHUB_SLAVE1_TAG;
        break;
    case LSM6DSO_SENSORHUB_SLAVE2_TAG:
        *val = LSM6DSO_SENSORHUB_SLAVE2_TAG;
        break;
    case LSM6DSO_SENSORHUB_SLAVE3_TAG:
        *val = LSM6DSO_SENSORHUB_SLAVE3_TAG;
        break;
    case LSM6DSO_STEP_CPUNTER_TAG:
        *val = LSM6DSO_STEP_CPUNTER_TAG;
        break;
    case LSM6DSO_GAME_ROTATION_TAG:
        *val = LSM6DSO_GAME_ROTATION_TAG;
        break;
    case LSM6DSO_GEOMAG_ROTATION_TAG:
        *val = LSM6DSO_GEOMAG_ROTATION_TAG;
        break;
    case LSM6DSO_ROTATION_TAG:
        *val = LSM6DSO_ROTATION_TAG;
        break;
    case LSM6DSO_SENSORHUB_NACK_TAG:
        *val = LSM6DSO_SENSORHUB_NACK_TAG;
        break;
    default:
        *val = LSM6DSO_GYRO_NC_TAG;
        break;
    }
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_fifo_out_raw_get(lsm6dso_t *lsm, int16_t *buff)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_X_L, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_X_H, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = (reg << 8) | buff[0];
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_Y_L, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_Y_H, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = (reg << 8) | buff[1];
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_Z_L, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = reg;
    CHECK(lsm->spi_read(LSM6DSO_FIFO_DATA_OUT_Z_H, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = (reg << 8) | buff[2];
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_acceleration_raw_get(lsm6dso_t *lsm, int16_t *buff)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTX_L_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTX_H_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = (reg << 8) | buff[0];
    CHECK(lsm->spi_read(LSM6DSO_OUTY_L_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTY_H_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = (reg << 8) | buff[1];
    CHECK(lsm->spi_read(LSM6DSO_OUTZ_L_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTZ_H_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = (reg << 8) | buff[2];
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_gyro_rate_raw_get(lsm6dso_t *lsm, int16_t *buff)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTX_L_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTX_H_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[0] = (reg << 8) | buff[0];
    CHECK(lsm->spi_read(LSM6DSO_OUTY_L_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTY_H_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[1] = (reg << 8) | buff[1];
    CHECK(lsm->spi_read(LSM6DSO_OUTZ_L_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = reg;
    CHECK(lsm->spi_read(LSM6DSO_OUTZ_H_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    buff[2] = (reg << 8) | buff[2];
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_acc_self_test_set(lsm6dso_t *lsm, lsm6dso_st_acc_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL5_C, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL5_C_ST_XL_MASK);
    reg |= (uint8_t)val;
    CHECK(lsm->spi_write(LSM6DSO_CTRL5_C, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_gyro_self_test_set(lsm6dso_t *lsm, lsm6dso_st_gyro_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL5_C, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL5_C_ST_G_MASK);
    reg |= (uint8_t)val << 2;
    CHECK(lsm->spi_write(LSM6DSO_CTRL5_C, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_acc_flag_data_ready_get(lsm6dso_t *lsm, uint8_t *val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_STATUS_REG, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    *val = reg & 0x01;
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_gyro_flag_data_ready_get(lsm6dso_t *lsm, uint8_t *val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_STATUS_REG, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    *val = (reg >> 1) & 0x01;
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_toggle_sensor(lsm6dso_t *lsm, uint8_t val)
{
    if (val > 0)
    {
        uint8_t reg_5;
        CHECK(lsm->spi_read(LSM6DSO_CTRL5_C, &reg_5, 1) == LSM6DSO_OK, LSM6DSO_ERR);
        reg_5 &= ~(0x80);
        CHECK(lsm->spi_write(LSM6DSO_CTRL5_C, reg_5) == LSM6DSO_OK, LSM6DSO_ERR);
        uint8_t reg_4;
        CHECK(lsm->spi_read(LSM6DSO_CTRL4_C, &reg_4, 1) == LSM6DSO_OK, LSM6DSO_ERR);
        reg_4 &= ~(0x40);
        CHECK(lsm->spi_write(LSM6DSO_CTRL4_C, reg_4) == LSM6DSO_OK, LSM6DSO_ERR);
    }
    else
    {
        uint8_t reg_5;
        CHECK(lsm->spi_read(LSM6DSO_CTRL5_C, &reg_5, 1) == LSM6DSO_OK, LSM6DSO_ERR);
        reg_5 &= ~(0x80);
        reg_5 |= (0x80);
        CHECK(lsm->spi_write(LSM6DSO_CTRL5_C, reg_5) == LSM6DSO_OK, LSM6DSO_ERR);
        uint8_t reg_4;
        CHECK(lsm->spi_read(LSM6DSO_CTRL4_C, &reg_4, 1) == LSM6DSO_OK, LSM6DSO_ERR);
        reg_4 &= ~(0x40);
        reg_4 |= (0x40);
        CHECK(lsm->spi_write(LSM6DSO_CTRL4_C, reg_4) == LSM6DSO_OK, LSM6DSO_ERR);
    }
    return LSM6DSO_OK;
}
lsm6dso_err_t lsm6dso_toggle_fifo(lsm6dso_t *lsm, uint8_t val)
{
    if (val > 0)
    {
        CHECK(lsm6dso_fifo_mode_set(lsm, LSM6DSO_STREAM_MODE) == LSM6DSO_OK, LSM6DSO_ERR);
    }
    else
    {
        CHECK(lsm6dso_fifo_mode_set(lsm, LSM6DSO_BYPASS_MODE) == LSM6DSO_OK, LSM6DSO_ERR);
    }
    return LSM6DSO_OK;
}

int lsm6dso_read_fifo(lsm6dso_t *lsm)
{
    uint16_t num = 0;
    int16_t data_raw[3] = {0};
    int16_t dummy[3] = {0};
    lsm6dso_fifo_tag_t reg_tag;
    uint8_t count_acc = 0;
    uint8_t count_gyro = 0;
    /* Read number of samples in FIFO */
    lsm6dso_fifo_data_level_get(lsm, &num);
    for (int i = 0; i < num; i++)
    {
        lsm6dso_fifo_sensor_tag_get(lsm, &reg_tag);
        switch (reg_tag)
        {
        case LSM6DSO_XL_NC_TAG:
            memset(data_raw, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(lsm, data_raw);
            lsm->fifo_acc[count_acc++] = lsm6dso_from_fs4_to_g(data_raw[0]);
            lsm->fifo_acc[count_acc++] = lsm6dso_from_fs4_to_g(data_raw[1]);
            lsm->fifo_acc[count_acc++] = lsm6dso_from_fs4_to_g(data_raw[2]);
            break;
        case LSM6DSO_GYRO_NC_TAG:
            memset(data_raw, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(lsm, data_raw);
            lsm->fifo_gyro[count_gyro++] = lsm6dso_from_fs500_to_dps(data_raw[0]);
            lsm->fifo_gyro[count_gyro++] = lsm6dso_from_fs500_to_dps(data_raw[1]);
            lsm->fifo_gyro[count_gyro++] = lsm6dso_from_fs500_to_dps(data_raw[2]);
            break;
        default:
            memset(dummy, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(lsm, dummy);
            break;
        }
    }
    return num;
}
float_t lsm6dso_from_fs2_to_g(int16_t lsb)
{
    return (((float_t)lsb) * 0.061f) / 1000;
}
float_t lsm6dso_from_fs4_to_g(int16_t lsb)
{
    return (((float_t)lsb) * 0.122f) / 1000;
}
float_t lsm6dso_from_fs8_to_g(int16_t lsb)
{
    return (((float_t)lsb) * 0.244f) / 1000;
}
float_t lsm6dso_from_fs16_to_g(int16_t lsb)
{
    return (((float_t)lsb) * 0.488f) / 1000;
}
float_t lsm6dso_from_fs125_to_dps(int16_t lsb)
{
    return (((float_t)lsb) * 4.375f) / 1000;
}
float_t lsm6dso_from_fs500_to_dps(int16_t lsb)
{
    return (((float_t)lsb) * 17.50f) / 1000;
}
float_t lsm6dso_from_fs250_to_dps(int16_t lsb)
{
    return (((float_t)lsb) * 8.750f) / 1000;
}
float_t lsm6dso_from_fs1000_to_dps(int16_t lsb)
{
    return (((float_t)lsb) * 35.0f) / 1000;
}
float_t lsm6dso_from_fs2000_to_dps(int16_t lsb)
{
    return (((float_t)lsb) * 70.0f) / 1000;
}
/* Private Function definitions ---------------------------------------- */
static lsm6dso_err_t lsm6dso_whoami(lsm6dso_t *lsm)
{
    uint8_t whoami;
    CHECK(lsm->spi_read(LSM6DSO_WHO_AM_I, &whoami, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    CHECK(whoami == LSM6DSO_ID, LSM6DSO_ERR);
    printf("Who Am I: 0x%x\n", whoami);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_reset_set(lsm6dso_t *lsm, uint8_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL3_C, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL3_C_SW_RESET_MASK);
    reg |= val;
    CHECK(lsm->spi_write(LSM6DSO_CTRL3_C, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_block_data_update_set(lsm6dso_t *lsm, uint8_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL3_C, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL3_C_BDU_MASK);
    reg |= val << 6;
    CHECK(lsm->spi_write(LSM6DSO_CTRL3_C, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_acc_full_scale_set(lsm6dso_t *lsm, lsm6dso_fs_acc_t val)
{

    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL1_XL, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL1_XL_FS_XL_MASK);
    reg |= (uint8_t)val << 2;
    CHECK(lsm->spi_write(LSM6DSO_CTRL1_XL, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_gyro_full_scale_set(lsm6dso_t *lsm, lsm6dso_fs_gyro_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL2_G, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL2_G_FS_G_MASK);
    reg |= (uint8_t)val << 1;
    CHECK(lsm->spi_write(LSM6DSO_CTRL2_G, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_i3c_disable_set(lsm6dso_t *lsm, uint8_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_CTRL9_XL, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_CTRL9_XL_I3C_DISABLE_MASK);
    reg |= val << 1;
    CHECK(lsm->spi_write(LSM6DSO_CTRL9_XL, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_embedd_set(lsm6dso_t *lsm, lsm6dso_reg_access_t val)
{
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_FUNC_CFG_ACCESS, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg &= ~(LSM6DSO_FUNC_CFG_ACCESS_REG_ACCESS_MASK);
    reg |= (uint8_t)val << 6;
    CHECK(lsm->spi_write(LSM6DSO_FUNC_CFG_ACCESS, reg) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_fsm_enable_get(lsm6dso_t *lsm, uint16_t *val)
{
    uint8_t reg;
    uint8_t data[2];
    CHECK(lsm->spi_read(LSM6DSO_FSM_ENABLE_A, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    data[0] = reg;
    CHECK(lsm->spi_read(LSM6DSO_FSM_ENABLE_B, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    data[1] = reg;
    *val = (data[0] << 8) | data[1];
    CHECK(lsm6dso_embedd_set(lsm, LSM6DSO_USER_BANK) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_fsm_data_rate_get(lsm6dso_t *lsm, lsm6dso_fsm_odr_t *val)
{
    uint8_t reg_fsm_odr;
    CHECK(lsm6dso_embedd_set(lsm, LSM6DSO_EMBEDDED_FUNC_BANK) == LSM6DSO_OK, LSM6DSO_ERR);
    uint8_t reg;
    CHECK(lsm->spi_read(LSM6DSO_EMB_FUNC_ODR_CFG_B, &reg, 1) == LSM6DSO_OK, LSM6DSO_ERR);
    reg_fsm_odr = ((reg & 0x18) >> 3);
    switch (reg_fsm_odr)
    {
    case LSM6DSO_ODR_FSM_12HZ5:
        *val = LSM6DSO_ODR_FSM_12HZ5;
        break;
    case LSM6DSO_ODR_FSM_26HZ:
        *val = LSM6DSO_ODR_FSM_26HZ;
        break;
    case LSM6DSO_ODR_FSM_52HZ:
        *val = LSM6DSO_ODR_FSM_52HZ;
        break;
    case LSM6DSO_ODR_FSM_104HZ:
        *val = LSM6DSO_ODR_FSM_104HZ;
        break;
    default:
        *val = LSM6DSO_ODR_FSM_12HZ5;
        break;
    }
    CHECK(lsm6dso_embedd_set(lsm, LSM6DSO_USER_BANK) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
static lsm6dso_err_t lsm6dso_self_test(lsm6dso_t *lsm)
{
    int16_t data_raw[3];
    float val_st_off[3];
    float val_st_on[3];
    float test_val[3];
    uint8_t st_result;
    uint8_t drdy;
    uint8_t i;
    uint8_t j;
    /* Self test acceleration */
    lsm6dso_acc_data_rate_set(lsm, LSM6DSO_XL_ODR_104HZ);
    do
    {
        lsm6dso_acc_flag_data_ready_get(lsm, &drdy);
    } while (!drdy);
    lsm6dso_acceleration_raw_get(lsm, data_raw);
    memset(val_st_off, 0x00, 3 * sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            lsm6dso_acc_flag_data_ready_get(lsm, &drdy);
        } while (!drdy);

        /* Read data and accumulate the g value */
        lsm6dso_acceleration_raw_get(lsm, data_raw);

        for (j = 0; j < 3; j++)
        {
            val_st_off[j] += lsm6dso_from_fs4_to_g(data_raw[j]);
        }
    }
    /* Calculate the g average values */
    for (i = 0; i < 3; i++)
    {
        val_st_off[i] /= 5.0f;
    }
    /* Enable Self Test positive (or negative) */
    lsm6dso_acc_self_test_set(lsm, LSM6DSO_XL_ST_NEGATIVE);
    do
    {
        lsm6dso_acc_flag_data_ready_get(lsm, &drdy);
    } while (!drdy);
    /* Read dummy data and discard it */
    lsm6dso_acceleration_raw_get(lsm, data_raw);
    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_on, 0x00, 3 * sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            lsm6dso_acc_flag_data_ready_get(lsm, &drdy);
        } while (!drdy);
        /* Read data and accumulate the mg value */
        lsm6dso_acceleration_raw_get(lsm, data_raw);
        for (j = 0; j < 3; j++)
        {
            val_st_on[j] += lsm6dso_from_fs4_to_g(data_raw[j]);
        }
    }
    /* Calculate the g average values */
    for (i = 0; i < 3; i++)
    {
        val_st_on[i] /= 5.0f;
    }
    /* Calculate the g values for self test */
    for (i = 0; i < 3; i++)
    {
        test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
    }
    /* Check self test limit */
    st_result = ST_PASS;
    for (i = 0; i < 3; i++)
    {
        if ((MIN_ST_LIMIT_G > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_G))
        {
            st_result = ST_FAIL;
        }
    }
    /* Disable Self Test */
    lsm6dso_acc_self_test_set(lsm, LSM6DSO_XL_ST_DISABLE);
    CHECK(st_result == ST_PASS, LSM6DSO_ERR);
    printf("\n---------------\nSelf test averages:\n    AX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n\n",
           (uint16_t)val_st_on[0], (uint16_t)val_st_on[1], (uint16_t)val_st_on[2]);
    printf("Measured averages:\n    AX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n---------------\n\n",
           (uint16_t)val_st_off[0], (uint16_t)val_st_off[1], (uint16_t)val_st_off[2]);
    CHECK(lsm6dso_gyro_full_scale_set(lsm, LSM6DSO_2000DPS) == LSM6DSO_OK, LSM6DSO_ERR);
    lsm6dso_gyro_data_rate_set(lsm, LSM6DSO_GY_ODR_104HZ);
    /* Self test gyro */
    do
    {
        lsm6dso_gyro_flag_data_ready_get(lsm, &drdy);
    } while (!drdy);
    /* Read dummy data and discard it */
    lsm6dso_gyro_rate_raw_get(lsm, data_raw);
    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_off, 0x00, 3 * sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            lsm6dso_gyro_flag_data_ready_get(lsm, &drdy);
        } while (!drdy);
        /* Read data and accumulate the dsp value */
        lsm6dso_gyro_rate_raw_get(lsm, data_raw);

        for (j = 0; j < 3; j++)
        {
            val_st_off[j] += lsm6dso_from_fs2000_to_dps(data_raw[j]);
        }
    }
    /* Calculate the dsp average values */
    for (i = 0; i < 3; i++)
    {
        val_st_off[i] /= 5.0f;
    }
    /* Enable Self Test positive (or negative) */
    lsm6dso_gyro_self_test_set(lsm, LSM6DSO_GY_ST_POSITIVE);
    memset(val_st_on, 0x00, 3 * sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            lsm6dso_gyro_flag_data_ready_get(lsm, &drdy);
        } while (!drdy);

        /* Read data and accumulate the dsp value */
        lsm6dso_gyro_rate_raw_get(lsm, data_raw);

        for (j = 0; j < 3; j++)
        {
            val_st_on[j] += lsm6dso_from_fs2000_to_dps(data_raw[j]);
        }
    }
    /* Calculate the dsp average values */
    for (i = 0; i < 3; i++)
    {
        val_st_on[i] /= 5.0f;
    }
    /* Calculate the dsp values for self test */
    for (i = 0; i < 3; i++)
    {
        test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
    }
    /* Check self test limit */
    for (i = 0; i < 3; i++)
    {
        if ((MIN_ST_LIMIT_DPS > test_val[i]) ||
            (test_val[i] > MAX_ST_LIMIT_DPS))
        {
            st_result = ST_FAIL;
        }
    }
    /* Disable Self Test */
    lsm6dso_gyro_self_test_set(lsm, LSM6DSO_GY_ST_DISABLE);
    CHECK(st_result == ST_PASS, LSM6DSO_ERR);
    printf("\n---------------\nSelf test averages:\n    GX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\n\n",
           (uint16_t)val_st_on[0], (uint16_t)val_st_on[1], (uint16_t)val_st_on[2]);
    printf("Measured averages:\n    GX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\n---------------\n\n",
           (uint16_t)val_st_off[0], (uint16_t)val_st_off[1], (uint16_t)val_st_off[2]);
    CHECK(lsm6dso_gyro_full_scale_set(lsm, LSM6DSO_500DPS) == LSM6DSO_OK, LSM6DSO_ERR);
    return LSM6DSO_OK;
}
/* End of file -------------------------------------------------------- */