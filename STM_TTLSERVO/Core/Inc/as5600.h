#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "main.h"
#include <stdint.h>

/* =============================================================================
 * AS5600 12-bit magnetic angle sensor  (I2C address 0x36)
 * ============================================================================= */

typedef enum {
    AS5600_OK          = 0,
    AS5600_ERR_I2C     = 1,   /* I2C communication failure */
    AS5600_ERR_NO_MAG  = 2,   /* Magnet not detected (MD bit clear) */
    AS5600_ERR_WEAK    = 3,   /* Magnet too weak  (ML bit set) */
    AS5600_ERR_STRONG  = 4,   /* Magnet too strong (MH bit set) */
} AS5600_Status_t;

/* Magnet status bits from the STATUS register (0x0B). */
typedef struct {
    uint8_t detected : 1;   /* MD: magnet in range */
    uint8_t too_weak : 1;   /* ML: AGC at maximum, magnet too weak */
    uint8_t too_strong : 1; /* MH: AGC at minimum, magnet too strong */
} AS5600_MagStatus_t;

/* =============================================================================
 * Public API
 * ============================================================================= */

/**
 * @brief  Check communication with AS5600 and verify magnet is in range.
 * @return AS5600_OK if the sensor replies and magnet is properly detected.
 */
AS5600_Status_t AS5600_Init(void);

/**
 * @brief  Read the filtered 12-bit angle (0–4095 = 0°–360°).
 * @param  angle  Output: angle value in [0, 4095].
 * @return AS5600_OK on success.
 */
AS5600_Status_t AS5600_ReadAngle(uint16_t *angle);

/**
 * @brief  Read the unfiltered raw 12-bit angle (0–4095 = 0°–360°).
 * @param  angle  Output: raw angle value in [0, 4095].
 * @return AS5600_OK on success.
 */
AS5600_Status_t AS5600_ReadRawAngle(uint16_t *angle);

/**
 * @brief  Read the magnet status flags.
 * @param  status  Output: struct with detected/too_weak/too_strong bits.
 * @return AS5600_OK on success, AS5600_ERR_I2C on communication failure.
 */
AS5600_Status_t AS5600_ReadMagStatus(AS5600_MagStatus_t *status);

/**
 * @brief  Read the automatic gain control value (0–255).
 *         Lower values = stronger magnet.  Optimal range: ~90–127.
 * @param  agc  Output: AGC register value.
 * @return AS5600_OK on success.
 */
AS5600_Status_t AS5600_ReadAGC(uint8_t *agc);

/**
 * @brief  Convert a raw 12-bit angle to degrees (0.0–359.91°).
 */
static inline float AS5600_ToDeg(uint16_t raw)
{
    return (float)raw * 360.0f / 4096.0f;
}

#endif /* INC_AS5600_H_ */
