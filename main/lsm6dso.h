/**
 * @file       lsm6dso.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-09-14
 * @author     Y Pham
 *
 * @brief       Driver for lsm6ds0
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __LSM6DSO_H_
#define __LSM6DSO_H_
/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#ifndef __FREERTOS__
#define __FREERTOS__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif /*__FREERTOS__*/
#include "driver/spi_master.h"
/* Public defines ----------------------------------------------------- */
#define LSM6DSO_FUNC_CFG_ACCESS              (0x01U) /*!< Enable embedded functions register */
#define LSM6DSO_FIFO_CTRL1                   (0x07U) /*!< FIFO control register */
#define LSM6DSO_PIN_CTRL                     (0x02U) /*!< SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register */
#define LSM6DSO_FIFO_CTRL2                   (0x08U) /*!< FIFO control register 2 */
#define LSM6DSO_FIFO_CTRL3                   (0x09U) /*!< FIFO control register 3 */
#define LSM6DSO_FIFO_CTRL4                   (0x0AU) /*!< FIFO control register 4 */
#define LSM6DSO_COUNTER_BDR_REG1             (0x0BU) /*!< Counter batch data rate register 1 */
#define LSM6DSO_COUNTER_BDR_REG2             (0x0CU) /*!< Counter batch data rate register 1 */
#define LSM6DSO_INT2_CTRL                    (0x0EU) /*!< INT2 pin control register */
#define LSM6DSO_WHO_AM_I                     (0x0FU) /*!< WHO_AM_I register  */
#define LSM6DSO_CTRL1_XL                     (0x10U) /*!< Accelerometer control register 1 */
#define LSM6DSO_CTRL2_G                      (0x11U) /*!< Gyroscope control register 2 */
#define LSM6DSO_CTRL3_C                      (0x12U) /*!< Control register 3 */
#define LSM6DSO_CTRL4_C                      (0x13U) /*!< Control register 4 */
#define LSM6DSO_CTRL5_C                      (0x14U) /*!< Control register 5 */
#define LSM6DSO_CTRL6_C                      (0x15U) /*!< Control register 6 */
#define LSM6DSO_CTRL7_G                      (0x16U) /*!< Control register 7 */
#define LSM6DSO_CTRL8_XL                     (0x17U) /*!< Control register 8 */
#define LSM6DSO_CTRL9_XL                     (0x18U) /*!< Control register 9 */
#define LSM6DSO_CTRL10_C                     (0x19U) /*!< Control register 10 */
#define LSM6DSO_ALL_INT_SRC                  (0x1AU) /*!< Source register for all interrupts */
#define LSM6DSO_WAKE_UP_SRC                  (0x1BU) /*!< Wake-up interrupt source register */
#define LSM6DSO_TAP_SRC                      (0x1CU) /*!< Tap source register */
#define LSM6DSO_D6D_SRC                      (0x1DU) /*!< Portrait, landscape, face-up and face-down source register */
#define LSM6DSO_STATUS_REG                   (0x1EU) /*!< The STATUS_REG register*/
#define LSM6DSO_STATUS_SPIAUX                (0x1EU) /*!< The primary interface SPI/I²C & MIPI I3CSM*/
#define LSM6DSO_OUT_TEMP_L                   (0x20U) /*!< Temperature data output register low*/
#define LSM6DSO_OUT_TEMP_H                   (0x21U) /*!< Temperature data output register hight*/
#define LSM6DSO_OUTX_L_G                     (0x22U) /*!< Angular rate sensor pitch axis (X) angular rate output register low */
#define LSM6DSO_OUTX_H_G                     (0x23U) /*!< Angular rate sensor pitch axis (X) angular rate output register high */
#define LSM6DSO_OUTY_L_G                     (0x24U) /*!< Angular rate sensor pitch axis (Y) angular rate output register low */
#define LSM6DSO_OUTY_H_G                     (0x25U) /*!< Angular rate sensor pitch axis (Y) angular rate output register high */
#define LSM6DSO_OUTZ_L_G                     (0x26U) /*!< Angular rate sensor pitch axis (Z) angular rate output register low */
#define LSM6DSO_OUTZ_H_G                     (0x27U) /*!< Angular rate sensor pitch axis (Z) angular rate output register high */
#define LSM6DSO_OUTX_L_A                     (0x28U) /*!< Acceleration rate sensor pitch axis (X) angular rate output register low */
#define LSM6DSO_OUTX_H_A                     (0x29U) /*!< Acceleration rate sensor pitch axis (X) angular rate output register high */
#define LSM6DSO_OUTY_L_A                     (0x2AU) /*!< Acceleration rate sensor pitch axis (Y) angular rate output register low */
#define LSM6DSO_OUTY_H_A                     (0x2BU) /*!< Acceleration rate sensor pitch axis (Y) angular rate output register high */
#define LSM6DSO_OUTZ_L_A                     (0x2CU) /*!< Acceleration rate sensor pitch axis (Z) angular rate output register low */
#define LSM6DSO_OUTZ_H_A                     (0x2DU) /*!< Acceleration rate sensor pitch axis (Z) angular rate output register high */
#define LSM6DSO_EMB_FUNC_STATUS_MAINPAGE     (0x35U) /*!< Embedded function status register */
#define LSM6DSO_FSM_STATUS_A_MAINPAGE        (0x36U) /*!< Finite State Machine status register */
#define LSM6DSO_FSM_STATUS_B_MAINPAGE        (0x37U) /*!< Finite State Machine status register */
#define LSM6DSO_STATUS_MASTER_MAINPAGE       (0x39U) /*!< Sensor hub source register */
#define LSM6DSO_FIFO_STATUS1                 (0x3AU) /*!< FIFO status register 1 */
#define LSM6DSO_FIFO_STATUS2                 (0x3B)  /*!< FIFO status register 2 */
#define LSM6DSO_TIMESTAMP0                   (0x40U) /*!< Timestamp first data output register */
#define LSM6DSO_TIMESTAMP1                   (0x41U) /*!< Timestamp first data output register */
#define LSM6DSO_TIMESTAMP2                   (0x42U) /*!< Timestamp first data output register */ 
#define LSM6DSO_TIMESTAMP3                   (0x43U) /*!< Timestamp first data output register */
#define LSM6DSO_TAP_CFG0                     (0x56U) /*!< Activity/inactivity functions, configuration of filtering, and tap recognition functions */
#define LSM6DSO_TAP_CFG1                     (0x57U) /*!< Tap configuration register*/
#define LSM6DSO_TAP_CFG2                     (0x58U) /*!< Enables interrupt and inactivity functions, and tap recognition functions */
#define LSM6DSO_TAP_THS_6D                   (0x59U) /*!< Portrait/landscape position and tap function threshold register */
#define LSM6DSO_INT_DUR2                     (0x5AU) /*!< Tap recognition function setting register */
#define LSM6DSO_WAKE_UP_THS                  (0x5BU) /*!< Single/double-tap selection and wake-up configuration*/
#define LSM6DSO_WAKE_UP_DUR                  (0x5CU) /*!< Free-fall, wakeup and sleep mode functions duration setting register */
#define LSM6DSO_FREE_FALL                    (0x5DU) /*!< Free-fall function duration setting register */
#define LSM6DSO_MD1_CFG                      (0x5EU) /*!< Functions routing on INT1 register */
#define LSM6DSO_MD2_CFG                      (0x5FU) /*!< Functions routing on INT2 register */
#define LSM6DSO_I3C_BUS_AVB                  (0x62U) /*!< I3C_BUS_AVB register */
#define LSM6DSO_INTERNAL_FREQ_FINE           (0x63U) /*!< Internal frequency register */
#define LSM6DSO_INT_OIS                      (0x6FU) /*!< OIS interrupt configuration register and accelerometer self-test enable setting */
#define LSM6DSO_CTRL1_OIS                    (0x70U) /*!< OIS configuration register */
#define LSM6DSO_CTRL2_OIS                    (0x71U) /*!< OIS configuration register. */
#define LSM6DSO_CTRL3_OIS                    (0x72U) /*!< OIS configuration register.*/
#define LSM6DSO_X_OFS_USR                    (0x73U) /*!< Accelerometer X-axis user offset correction */
#define LSM6DSO_Y_OFS_USR                    (0x74U) /*!< Accelerometer Y-axis user offset correction */
#define LSM6DSO_Z_OFS_USR                    (0x75U) /*!< Accelerometer Z-axis user offset correction */
#define LSM6DSO_FIFO_DATA_OUT_TAG            (0x78U) /*!< FIFO tag register */
#define LSM6DSO_FIFO_DATA_OUT_X_L            (0x79U) /*!< FIFO data output X */
#define LSM6DSO_FIFO_DATA_OUT_X_H            (0x7AU) /*!< FIFO data output X */
#define LSM6DSO_FIFO_DATA_OUT_Y_L            (0x7BU) /*!< FIFO data output Y */
#define LSM6DSO_FIFO_DATA_OUT_Y_H            (0x7CU) /*!< FIFO data output Y */
#define LSM6DSO_FIFO_DATA_OUT_Z_L            (0x7DU) /*!< FIFO data output Z */
#define LSM6DSO_FIFO_DATA_OUT_Z_H            (0x7EU) /*!< FIFO data output Z */
#define LSM6DSO_PAGE_SEL                     (0x02U) /*!< Enable advanced features dedicated page */
#define LSM6DSO_EMB_FUNC_EN_A                (0x04U) /*!< Embedded functions enable register */
#define LSM6DSO_EMB_FUNC_EN_B                (0x05U) /*!< Embedded functions enable register */
#define LSM6DSO_PAGE_ADDRESS                 (0x08U) /*!< Page address register */
#define LSM6DSO_PAGE_VALUE                   (0x09U) /*!< Page value register */
#define LSM6DSO_EMB_FUNC_INT1                (0x0AU) /*!< INT1 pin control register */
#define LSM6DSO_FSM_INT1_A                   (0x0BU) /*!< INT1 pin control register */
#define LSM6DSO_FSM_INT1_B                   (0x0CU) /*!< INT1 pin control register */
#define LSM6DSO_EMB_FUNC_INT2                (0x0EU) /*!< INT2 pin control register */
#define LSM6DSO_FSM_INT2_A                   (0x0FU) /*!< INT2 pin control register */
#define LSM6DSO_FSM_INT2_B                   (0x10U) /*!< INT2 pin control register */
#define LSM6DSO_EMB_FUNC_STATUS              (0x12U) /*!< Embedded function status register */
#define LSM6DSO_FSM_STATUS_A                 (0x13U) /*!< Finite State Machine status register */
#define LSM6DSO_FSM_STATUS_B                 (0x14U) /*!< Finite State Machine status register */
#define LSM6DSO_PAGE_RW                      (0x17U) /*!< Enable read and write mode of advanced features dedicated page */
#define LSM6DSO_EMB_FUNC_FIFO_CFG            (0x44U) /*!< Embedded functions batching configuration register */
#define LSM6DSO_FSM_ENABLE_B                 (0x47U) /*!< FSM enable register */
#define LSM6DSO_FSM_LONG_COUNTER_L           (0x48U) /*!< FSM long counter status register */
#define LSM6DSO_FSM_LONG_COUNTER_H           (0x49U) /*!< FSM long counter status register */
#define LSM6DSO_FSM_LONG_COUNTER_CLEAR       (0x4AU) /*!< FSM long counter reset register */
#define LSM6DSO_FSM_OUTS1                    (0x4CU) /*!< FSM1 output register */
#define LSM6DSO_FSM_OUTS2                    (0x4DU) /*!< FSM2 output register */
#define LSM6DSO_FSM_OUTS3                    (0x4EU) /*!< FSM3 output register */
#define LSM6DSO_FSM_OUTS4                    (0x4FU) /*!< FSM4 output register */
#define LSM6DSO_FSM_OUTS5                    (0x50U) /*!< FSM5 output register */
#define LSM6DSO_FSM_OUTS6                    (0x51U) /*!< FSM6 output register */
#define LSM6DSO_FSM_OUTS7                    (0x52U) /*!< FSM7 output register */
#define LSM6DSO_FSM_OUTS8                    (0x53U) /*!< FSM8 output register */
#define LSM6DSO_FSM_OUTS9                    (0x54U) /*!< FSM9 output register */
#define LSM6DSO_FSM_OUTS10                   (0x55U) /*!< FSM10 output register */
#define LSM6DSO_FSM_OUTS11                   (0x56U) /*!< FSM11 output register */
#define LSM6DSO_FSM_OUTS12                   (0x57U) /*!< FSM12 output register */
#define LSM6DSO_FSM_OUTS13                   (0x58U) /*!< FSM13 output register */
#define LSM6DSO_FSM_OUTS14                   (0x59U) /*!< FSM14 output register */
#define LSM6DSO_FSM_OUTS15                   (0x5AU) /*!< FSM15 output register */
#define LSM6DSO_FSM_OUTS16                   (0x5BU) /*!< FSM16 output register */
#define LSM6DSO_EMB_FUNC_ODR_CFG_B           (0x5FU) /*!< Finite State Machine output data rate configuration register */
#define LSM6DSO_STEP_COUNTER_L               (0x62U) /*!< Step counter output register */
#define LSM6DSO_STEP_COUNTER_H               (0x63U) /*!< Step counter output register */
#define LSM6DSO_EMB_FUNC_SRC                 (0x64U) /*!< Embedded function source register */
#define LSM6DSO_EMB_FUNC_INIT_A              (0x66U) /*!< Embedded functions initialization register */
#define LSM6DSO_EMB_FUNC_INIT_B              (0x67U) /*!< Embedded functions initialization register */
#define LSM6DSO_MAG_SENSITIVITY_L            (0xBAU) /*!< External magnetometer sensitivity value register for the Finite State Machine */
#define LSM6DSO_MAG_SENSITIVITY_H            (0xBBU) /*!< External magnetometer sensitivity value register */
#define LSM6DSO_MAG_OFFX_L                   (0xC0U) /*!< Offset for X-axis hard-iron compensation register */
#define LSM6DSO_MAG_OFFX_H                   (0xC1U) /*!< Offset for X-axis hard-iron compensation register */
#define LSM6DSO_MAG_OFFY_L                   (0xC2U) /*!< Offset for Y-axis hard-iron compensation register */
#define LSM6DSO_MAG_OFFY_H                   (0xC3U) /*!< Offset for Y-axis hard-iron compensation register */
#define LSM6DSO_MAG_OFFZ_L                   (0xC4U) /*!< Offset for Z-axis hard-iron compensation register */
#define LSM6DSO_MAG_OFFZ_H                   (0xC5U) /*!< Offset for Z-axis hard-iron compensation register */
#define LSM6DSO_MAG_SI_XX_L                  (0xC6U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_XX_H                  (0xC7U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_XY_L                  (0xC8U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_XY_H                  (0xC9U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_XZ_L                  (0xCAU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_XZ_H                  (0xCBU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_YY_L                  (0xCCU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_YY_H                  (0xCDU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_YZ_L                  (0xCEU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_YZ_H                  (0xCFU) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_ZZ_L                  (0xD0U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_SI_ZZ_H                  (0xD1U) /*!< Soft-iron (3x3 symmetric) matrix correction register */
#define LSM6DSO_MAG_CFG_A                    (0xD4U) /*!< External magnetometer coordinates (Y and Z axes) rotation register */
#define LSM6DSO_MAG_CFG_B                    (0xD5U) /*!< External magnetometer coordinates (X-axis) rotation register */
#define LSM6DSO_FSM_LC_TIMEOUT_L             (0x7AU) /*!< FSM long counter timeout register */
#define LSM6DSO_FSM_LC_TIMEOUT_H             (0x7BU) /*!< FSM long counter timeout register */
#define LSM6DSO_FSM_PROGRAMS                 (0x7CU) /*!< FSM number of programs register */
#define LSM6DSO_FSM_START_ADD_L              (0x7EU) /*!< FSM start address register */
#define LSM6DSO_FSM_START_ADD_H              (0x7FU) /*!< FSM start address register */
#define LSM6DSO_PEDO_CMD_REG                 (0x83U) /*!< Pedometer configuration register */
#define LSM6DSO_PEDO_DEB_STEPS_CONF          (0x84U) /*!< Pedometer debounce configuration register */
#define LSM6DSO_PEDO_SC_DELTAT_L             (0xD0U) /*!< Time period register for step detection on delta time */
#define LSM6DSO_PEDO_SC_DELTAT_H             (0xD1U) /*!< Time period register for step detection on delta time */
#define LSM6DSO_FSM_ENABLE_A                 (0x46U) /*!< FSM enable register*/
#define LSM6DSO_ID							             (0x6C)  /*!< Chip ID*/
/**
 * @brief Block Data Update. Default value: 0
 *
 * @param   0:      continuous update
 * @param   1:      output registers are not updated until MSB and LSB have been read
 */
#define LSM6DSO_CTRL3_C_BDU_MASK (0x40)

/**
 * @brief Accelerometer full-scale selection
 *
 * @param	00:					+/-2g
 * @param	01:					+/-16g
 * @param	10:					+/-4g
 * @param	11:					+/-8g
 */
#define LSM6DSO_CTRL1_XL_FS_XL_MASK (0x0C)

/**
 * @brief Gyroscope UI chain full-scale selection
 *
 * @param	00:					250 dps
 * @param	01:					500 dps
 * @param	10:					1000 dps
 * @param	11:					2000 dps
 */
#define LSM6DSO_CTRL2_G_FS_G_MASK (0x0E)

/**
 * @brief Disables MIPI I3CSM communication protocol
 *
 * @param   0:       SPI, I²C, MIPI I3CSM interfaces enabled
 * @param   1:       MIPI I3CSM interface disabled
 */
#define LSM6DSO_CTRL9_XL_I3C_DISABLE_MASK (0x02)

/**
 * @brief Selects Batching Data Rate (writing frequency in FIFO) for accelerometer data
 *
 * @param   0000:   Accelerometer not batched in FIFO
 * @param 	0001:		12.5 Hz
 * @param	  0010:		26 Hz
 * @param	  0011:		52 Hz
 * @param	  0100:		104 Hz
 * @param	  0101:		208 Hz
 * @param	  0110:		417 Hz
 * @param	  0111:		833 Hz
 * @param	  1000:		1667 Hz
 * @param	  1001:		3333 Hz
 * @param	  1010:		6667 Hz
 * @param	  1011:		1.6Hz
 * @param	  1100 -> 1111: not allowed
 *
 */
#define LSM6DSO_FIFO_CTRL3_BDR_XL_MASK (0x0F)

/**
 * @brief Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
 *
 * @param   0000:   Gyro not batched in FIFO
 * @param 	0001:		12.5 Hz
 * @param	  0010:		26 Hz
 * @param	  0011:		52 Hz
 * @param	  0100:		104 Hz
 * @param	  0101:		208 Hz
 * @param	  0110:		417 Hz
 * @param	  0111:		833 Hz
 * @param	  1000:		1667 Hz
 * @param	  1001:		3333 Hz
 * @param	  1010:		6667 Hz
 * @param	  1011:		1.6Hz
 * @param	  1100 - 1111:not allowed
 *
 */
#define LSM6DSO_FIFO_CTRL3_BDR_GY_MASK (0xF0)

/**
 * @brief FIFO mode selection
 *
 * @param 000:    Bypass mode: FIFO disabled
 * @param 001:		FIFO mode: stops collecting data when FIFO is full
 * @param	010:		Reserved
 * @param	011:		Continuous-to-FIFO mode
 * @param	100:		Bypass-to-Continuous mode
 * @param	101:		Reserved
 * @param	110:		Continuous mode/Stream mode
 * @param	111:		Bypass-to-FIFO mode
 *
 */
#define LSM6DSO_FIFO_CTRL4_FIFO_MODE_MASK (0x07)

/**
 * @brief 	FIFO watermark threshold, in conjunction with WTM_FIFO[7:0] in FIFO_CTRL1 (07h)
 * 			1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO
 * 			Watermark flag rises when the number of bytes written in the FIFO
 * 			is greater than or equal to the threshold level.
 *
 */
#define LSM6DSO_FIFO_CTRL2_WTM_MASK (0x01)

/**
 * @brief FIFO watermark threshold, in conjunction with WTM8 in FIFO_CTRL2 (08h)
 * 1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO
 * Watermark flag rises when the number of bytes written in the FIFO is greater than or equal to the threshold level
 *
 */
#define LSM6DSO_FIFO_CTRL1_WTM_MASK (0xFF)

/**
 * @brief shub_reg_access + func_cfg_access
 *
 */
#define LSM6DSO_FUNC_CFG_ACCESS_REG_ACCESS_MASK (0xC0)

/**
 * @brief Accelerometer ODR selection
 *
 */
#define LSM6DSO_CTRL1_XL_ORD_XL_MASK (0xF0)

#define LSM6DSO_CTRL2_G_ODR_G_MASK (0xF0)

#define LSM6DSO_CTRL3_C_SW_RESET_MASK (0x01)

#define LSM6DSO_CTRL5_C_ST_XL_MASK (0x03)

#define LSM6DSO_CTRL5_C_ST_G_MASK (0x0C)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  LSM6DSO_OK,
  LSM6DSO_ERR,
  LSM6DSO_ERR_PARAM
} lsm6dso_err_t;
typedef enum
{
  LSM6DSO_2G  = 0,
  LSM6DSO_16G = 1,
  LSM6DSO_4G  = 2,
  LSM6DSO_8G  = 3,
} lsm6dso_fs_acc_t;
typedef enum
{
  LSM6DSO_250DPS  = 0,
  LSM6DSO_125DPS  = 1,
  LSM6DSO_500DPS  = 2,
  LSM6DSO_1000DPS = 4,
  LSM6DSO_2000DPS = 6,
} lsm6dso_fs_gyro_t;
typedef enum
{
  LSM6DSO_XL_ODR_OFF    = 0,
  LSM6DSO_XL_ODR_12HZ5  = 1,
  LSM6DSO_XL_ODR_26HZ   = 2,
  LSM6DSO_XL_ODR_52HZ   = 3,
  LSM6DSO_XL_ODR_104HZ  = 4,
  LSM6DSO_XL_ODR_208HZ  = 5,
  LSM6DSO_XL_ODR_417HZ  = 6,
  LSM6DSO_XL_ODR_833HZ  = 7,
  LSM6DSO_XL_ODR_1HZ667 = 8,
  LSM6DSO_XL_ODR_3HZ333 = 9,
  LSM6DSO_XL_ODR_6HZ667 = 10,
  LSM6DSO_XL_ODR_1HZ6   = 11, /* (low power only) */
} lsm6dso_odr_acc_t;

typedef enum
{
  LSM6DSO_GY_ODR_OFF    = 0,
  LSM6DSO_GY_ODR_12HZ5  = 1,
  LSM6DSO_GY_ODR_26HZ   = 2,
  LSM6DSO_GY_ODR_52HZ   = 3,
  LSM6DSO_GY_ODR_104HZ  = 4,
  LSM6DSO_GY_ODR_208HZ  = 5,
  LSM6DSO_GY_ODR_417HZ  = 6,
  LSM6DSO_GY_ODR_833HZ  = 7,
  LSM6DSO_GY_ODR_1667HZ = 8,
  LSM6DSO_GY_ODR_3333HZ = 9,
  LSM6DSO_GY_ODR_6667HZ = 10,
} lsm6dso_odr_gyro_t;

typedef enum
{
  LSM6DSO_USER_BANK           = 0,
  LSM6DSO_SENSOR_HUB_BANK     = 1,
  LSM6DSO_EMBEDDED_FUNC_BANK  = 2,
} lsm6dso_reg_access_t;

typedef enum
{
  LSM6DSO_ODR_FSM_12HZ5   = 0,
  LSM6DSO_ODR_FSM_26HZ    = 1,
  LSM6DSO_ODR_FSM_52HZ    = 2,
  LSM6DSO_ODR_FSM_104HZ   = 3,
} lsm6dso_fsm_odr_t;

typedef enum
{
  LSM6DSO_GYRO_NC_TAG               = 1,
  LSM6DSO_XL_NC_TAG,
  LSM6DSO_TEMPERATURE_TAG,
  LSM6DSO_TIMESTAMP_TAG,
  LSM6DSO_CFG_CHANGE_TAG,
  LSM6DSO_XL_NC_T_2_TAG,
  LSM6DSO_XL_NC_T_1_TAG,
  LSM6DSO_XL_2XC_TAG,
  LSM6DSO_XL_3XC_TAG,
  LSM6DSO_GYRO_NC_T_2_TAG,
  LSM6DSO_GYRO_NC_T_1_TAG,
  LSM6DSO_GYRO_2XC_TAG,
  LSM6DSO_GYRO_3XC_TAG,
  LSM6DSO_SENSORHUB_SLAVE0_TAG,
  LSM6DSO_SENSORHUB_SLAVE1_TAG,
  LSM6DSO_SENSORHUB_SLAVE2_TAG,
  LSM6DSO_SENSORHUB_SLAVE3_TAG,
  LSM6DSO_STEP_CPUNTER_TAG,
  LSM6DSO_GAME_ROTATION_TAG,
  LSM6DSO_GEOMAG_ROTATION_TAG,
  LSM6DSO_ROTATION_TAG,
  LSM6DSO_SENSORHUB_NACK_TAG        = 0x19,
} lsm6dso_fifo_tag_t;

typedef enum
{
  LSM6DSO_XL_NOT_BATCHED        = 0,
  LSM6DSO_XL_BATCHED_AT_12HZ5   = 1,
  LSM6DSO_XL_BATCHED_AT_26HZ    = 2,
  LSM6DSO_XL_BATCHED_AT_52HZ    = 3,
  LSM6DSO_XL_BATCHED_AT_104HZ   = 4,
  LSM6DSO_XL_BATCHED_AT_208HZ   = 5,
  LSM6DSO_XL_BATCHED_AT_417HZ   = 6,
  LSM6DSO_XL_BATCHED_AT_833HZ   = 7,
  LSM6DSO_XL_BATCHED_AT_1667HZ  = 8,
  LSM6DSO_XL_BATCHED_AT_3333HZ  = 9,
  LSM6DSO_XL_BATCHED_AT_6667HZ  = 10,
  LSM6DSO_XL_BATCHED_AT_1HZ6    = 11,
} lsm6dso_bdr_acc_t;
typedef enum
{
  LSM6DSO_GY_NOT_BATCHED        = 0,
  LSM6DSO_GY_BATCHED_AT_12HZ5   = 1,
  LSM6DSO_GY_BATCHED_AT_26HZ    = 2,
  LSM6DSO_GY_BATCHED_AT_52HZ    = 3,
  LSM6DSO_GY_BATCHED_AT_104HZ   = 4,
  LSM6DSO_GY_BATCHED_AT_208HZ   = 5,
  LSM6DSO_GY_BATCHED_AT_417HZ   = 6,
  LSM6DSO_GY_BATCHED_AT_833HZ   = 7,
  LSM6DSO_GY_BATCHED_AT_1667HZ  = 8,
  LSM6DSO_GY_BATCHED_AT_3333HZ  = 9,
  LSM6DSO_GY_BATCHED_AT_6667HZ  = 10,
  LSM6DSO_GY_BATCHED_AT_6HZ5    = 11,
} lsm6dso_bdr_gyro_t;

typedef enum
{
  LSM6DSO_BYPASS_MODE           = 0,
  LSM6DSO_FIFO_MODE             = 1,
  LSM6DSO_STREAM_TO_FIFO_MODE   = 3,
  LSM6DSO_BYPASS_TO_STREAM_MODE = 4,
  LSM6DSO_STREAM_MODE           = 6,
  LSM6DSO_BYPASS_TO_FIFO_MODE   = 7,
} lsm6dso_fifo_mode_t;

typedef enum
{
  LSM6DSO_XL_ST_DISABLE   = 0,
  LSM6DSO_XL_ST_POSITIVE  = 1,
  LSM6DSO_XL_ST_NEGATIVE  = 2,
} lsm6dso_st_acc_t;

typedef enum
{
  LSM6DSO_GY_ST_DISABLE   = 0,
  LSM6DSO_GY_ST_POSITIVE  = 1,
  LSM6DSO_GY_ST_NEGATIVE  = 3,
} lsm6dso_st_gyro_t;

typedef struct 
{
  lsm6dso_err_t (*spi_read)(uint8_t register_address, uint8_t *value, uint8_t len);
  lsm6dso_err_t (*spi_write)(uint8_t register_address, uint8_t value);

  float fifo_acc[512*3];
  float fifo_gyro[512*3];
  
   float acc_x_off;										   /**< Accelerometer offset values */ 
	 float acc_y_off;
	 float acc_z_off;

	 float gyro_x_off;										 /**< Gyroscope offset values */
	 float gyro_y_off;
	 float gyro_z_off;
} lsm6dso_t;
/* Public macros ------------------------------------------------------ */
/**
 * @brief Check error
 * 
 */
#define CHECK(_A_, _OUT_) \
    do                    \
    {                     \
        if (!(_A_))       \
            return _OUT_; \
    } while (0)
/* Function definitions ----------------------------------------------- */
/**
 * @brief Initialize LSM6DSO
 *
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_init(lsm6dso_t *lsm);
/**
 * @brief  Selects Batching Data Rate (writing frequency in FIFO) for accelerometer data.[set]
 *
 * @param[in]  val      change the values of bdr_xl in reg FIFO_CTRL3
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_acc_batch_set(lsm6dso_t *lsm, lsm6dso_bdr_acc_t val);

/**
 * @brief  Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.[set]
 *
 * @param[in]  val      change the values of bdr_gy in reg FIFO_CTRL3
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_fifo_gyro_batch_set(lsm6dso_t *lsm, lsm6dso_bdr_gyro_t val);

/**
 * @brief  FIFO mode selection.[set]
 *
 * @param[in]  val      change the values of fifo_mode in reg FIFO_CTRL4
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_mode_set(lsm6dso_t *lsm, lsm6dso_fifo_mode_t val);

/**
 * @brief  FIFO watermark level selection.[set]
 *
 * @param[in]  val      change the values of wtm in reg FIFO_CTRL1
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_watermark_set(lsm6dso_t *lsm, uint16_t val);

/**
 * @brief  Accelerometer UI data rate selection.[set]
 *
 * @param[in]  val      change the values of odr_xl in reg CTRL1_XL
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_acc_data_rate_set(lsm6dso_t *lsm, lsm6dso_odr_acc_t val);

/**
 * @brief  Gyroscope UI data rate selection.[set]
 *
 * @param[in]  val      change the values of odr_g in reg CTRL2_G
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_gyro_data_rate_set(lsm6dso_t *lsm, lsm6dso_odr_gyro_t val);

/**
 * @brief  FIFO watermark status.[get]
 *
 * @param[out]  val      change the values of fifo_wtm_ia in reg FIFO_STATUS2
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_fifo_wtm_flag_get(lsm6dso_t *lsm, uint8_t *val);

/**
 * @brief  Number of unread sensor data(TAG + 6 bytes) stored in FIFO.[get]
 *
 * @param[out]  val      change the values of diff_fifo in reg FIFO_STATUS1
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_data_level_get(lsm6dso_t *lsm, uint16_t *val);

/**
 * @brief  Identifies the sensor in FIFO_DATA_OUT.[get]
 *
 * @param[out]  val      change the values of tag_sensor in reg FIFO_DATA_OUT_TAG
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_sensor_tag_get(lsm6dso_t *lsm, lsm6dso_fifo_tag_t *val);

/**
 * @brief  FIFO data output [get]
 *
 * @param[out]  buff     buffer that stores data read
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_fifo_out_raw_get(lsm6dso_t *lsm, int16_t  *buff);
/**
 * @brief  Accelerometer new data available.[get]
 *
 * @param[out]  val      change the values of xlda in reg STATUS_REG
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_acc_flag_data_ready_get(lsm6dso_t *lsm, uint8_t *val);
/**
 * @brief  Linear acceleration output register. The value is expressed as a 16-bit word in two’s complement.[get]
 *
 * @param[out]  buff     buffer that stores data read
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_acceleration_raw_get(lsm6dso_t *lsm, int16_t *buff);
/**
 * @brief  Linear acceleration sensor self-test enable.[set]
 *
 * @param[in]  val      change the values of st_xl in reg CTRL5_C
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_acc_self_test_set(lsm6dso_t *lsm, lsm6dso_st_acc_t val);

/**
 * @brief  Gyroscope new data available.[get]
 *
 * @param[out]  val      change the values of gda in reg STATUS_REG
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_gyro_flag_data_ready_get(lsm6dso_t *lsm, uint8_t *val);

/**
 * @brief  Angular rate sensor. The value is expressed as a 16-bit word in two’s complement.[get]
 *
 * @param[out]  buff     buffer that stores data read
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_gyro_rate_raw_get(lsm6dso_t *lsm, int16_t *buff);

/**
 * @brief  Angular rate sensor self-test enable.[set]
 *
 * @param[in]  val      change the values of st_g in reg CTRL5_C
 * @return lsm6dso_err_t
 *
 */
lsm6dso_err_t lsm6dso_gyro_self_test_set(lsm6dso_t *lsm, lsm6dso_st_gyro_t val);

/**
 * @brief Toggle sensor
 *
 * @param[in] val 0: Turn off accel and gyro sensors
 *                1: Turn on accel and gyro sensors
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_toggle_sensor(lsm6dso_t *lsm, uint8_t val);
/**
 * @brief Toggle fifo
 *
 * @param[in] val 0: Turn off fifo
 *                1: Turn on fifo
 * @return lsm6dso_err_t
 */
lsm6dso_err_t lsm6dso_toggle_fifo(lsm6dso_t *lsm, uint8_t val);

/**
 * @brief Read FIFO
 *
 * @return  number samples in FIFO
 */
int lsm6dso_read_fifo(lsm6dso_t *lsm);
/**
 * @brief Convert 2g to g
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs2_to_g(int16_t lsb);
/**
 * @brief Convert 4g to g
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs4_to_g(int16_t lsb);
/**
 * @brief Convert 8g to g
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs8_to_g(int16_t lsb);
/**
 * @brief Convert 18g to g
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs16_to_g(int16_t lsb);
/**
 * @brief Convert 125dps to dps
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs125_to_dps(int16_t lsb);
/**
 * @brief Convert 500dps to dps
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs500_to_dps(int16_t lsb);
/**
 * @brief Convert 250dps to dps
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs250_to_dps(int16_t lsb);
/**
 * @brief Convert 1000dps to dps
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs1000_to_dps(int16_t lsb);
/**
 * @brief Convert 2000dps to dps
 * 
 * @param[in] lsb raw-data
 * @return  Engineering units
 */
float_t lsm6dso_from_fs2000_to_dps(int16_t lsb);
#endif /* __LSM6DSO_H_ */
/* End of file -------------------------------------------------------- */