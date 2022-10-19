/**
 * @file       bsp.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2022-10-06
 * @author     Y Pham
 *
 * @brief     BSP layer
 *
 */

#ifndef __BSP_H_
#define __BSP_H_
/* Includes ----------------------------------------------------------- */
#include "lsm6dso.h"
/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Function definitions ----------------------------------------------- */
/**
 * @brief Implementation of I2C read for 8-bit values
 * 
 * @param[in] register_address register address
 * @param[out] value Read value of register
 * @param[in] len  Total data length, in bits
 * @return lsm6dso_err_t 
 */
lsm6dso_err_t lsm6dso_read(uint8_t register_address, uint8_t *value, uint8_t len);
/**
 * @brief  Implementation of I2C write for 8-bit values
 * 
 * @param[in] register_address register address
 * @param[in] value Write value into register
 * @return lsm6dso_err_t 
 */
lsm6dso_err_t lsm6dso_write(uint8_t register_address, uint8_t value);
#endif /* __BSP_H_ */
/* End of file -------------------------------------------------------- */