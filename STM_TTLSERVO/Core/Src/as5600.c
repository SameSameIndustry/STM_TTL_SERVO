#include "as5600.h"
#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/* 7-bit address 0x36 shifted left for HAL (HAL appends R/W bit internally). */
#define AS5600_ADDR   (0x36u << 1)
#define I2C_TIMEOUT   10u   /* ms */

/* Register map */
#define REG_STATUS      0x0Bu
#define REG_RAW_ANGLE_H 0x0Cu
#define REG_ANGLE_H     0x0Eu
#define REG_AGC         0x1Au

/* STATUS register bit masks */
#define STATUS_MD  (1u << 5)   /* Magnet Detected */
#define STATUS_ML  (1u << 4)   /* Magnet too Low  */
#define STATUS_MH  (1u << 3)   /* Magnet too High */

/* =========================================================================
 * Private helper: read one or more consecutive registers.
 * ========================================================================= */
static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

AS5600_Status_t AS5600_Init(void)
{
    uint8_t status;
    if (reg_read(REG_STATUS, &status, 1) != HAL_OK)
        return AS5600_ERR_I2C;

    if (!(status & STATUS_MD)) return AS5600_ERR_NO_MAG;
    if (  status & STATUS_ML ) return AS5600_ERR_WEAK;
    if (  status & STATUS_MH ) return AS5600_ERR_STRONG;

    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadAngle(uint16_t *angle)
{
    uint8_t buf[2];
    if (reg_read(REG_ANGLE_H, buf, 2) != HAL_OK)
        return AS5600_ERR_I2C;

    *angle = ((uint16_t)(buf[0] & 0x0Fu) << 8) | buf[1];
    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadRawAngle(uint16_t *angle)
{
    uint8_t buf[2];
    if (reg_read(REG_RAW_ANGLE_H, buf, 2) != HAL_OK)
        return AS5600_ERR_I2C;

    *angle = ((uint16_t)(buf[0] & 0x0Fu) << 8) | buf[1];
    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadMagStatus(AS5600_MagStatus_t *status)
{
    uint8_t raw;
    if (reg_read(REG_STATUS, &raw, 1) != HAL_OK)
        return AS5600_ERR_I2C;

    status->detected   = (raw & STATUS_MD) ? 1u : 0u;
    status->too_weak   = (raw & STATUS_ML) ? 1u : 0u;
    status->too_strong = (raw & STATUS_MH) ? 1u : 0u;
    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadAGC(uint8_t *agc)
{
    if (reg_read(REG_AGC, agc, 1) != HAL_OK)
        return AS5600_ERR_I2C;
    return AS5600_OK;
}
