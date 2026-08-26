/**
 * Minimal driver for FEETECH STS-series serial bus servos (e.g. STS3215)
 * over the half-duplex TTL bus on J4, driven through USART1 + the MAX3485
 * (DE/~RE tied together on PA7).
 */
#ifndef FEETECH_STS_H
#define FEETECH_STS_H

#include "main.h"

void STS_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin);

/* Ping a servo; returns 1 if it responded, 0 on timeout/no response. */
uint8_t STS_Ping(uint8_t id);

/* Move to an absolute position (0-4095 over one mechanical turn).
 * time_ms: 0 = use goal speed only, otherwise the servo paces itself to
 * arrive in approximately time_ms. speed: 0 = servo's own max speed. */
void STS_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed);

/* Enable/disable torque (must be enabled for WritePosition to take effect). */
void STS_TorqueEnable(uint8_t id, uint8_t enable);

/* Cap the servo's output torque while staying in position control.
 * limit_permille: 0-1000 (0-100% of stall torque). RAM-only register, so
 * it is safe to call repeatedly and resets to the EEPROM default (addr 16)
 * on power-up. */
void STS_SetTorqueLimit(uint8_t id, uint16_t limit_permille);

/* Read back a 2-byte register (e.g. address 48, Torque Limit) for verification.
 * Returns 1 and stores the value in *out on success, 0 on timeout/error. */
uint8_t STS_ReadWord(uint8_t id, uint8_t addr, uint16_t *out);

/* Read the servo's current encoder position (0-4095).
 * Returns 1 and stores the value in *out on success, 0 on timeout/error. */
uint8_t STS_ReadPosition(uint8_t id, uint16_t *out);

#define STS_MODE_POSITION  0u  /* default: absolute position control */
#define STS_MODE_PWM       2u  /* PWM open-loop control (pseudo torque control) */

/* Switch the servo's Operation Mode (EEPROM, addr 33). Disables torque
 * first; torque is left disabled afterward - call STS_TorqueEnable(id, 1)
 * once ready to drive. */
void STS_SetMode(uint8_t id, uint8_t mode);

/* Only meaningful in STS_MODE_PWM: drive the motor open-loop at a duty
 * cycle, -1000..1000 (-100%..100% of max drive voltage). Sign sets
 * direction. This is NOT closed-loop current control - actual output
 * current/torque still varies with load and back-EMF - but unlike
 * Torque_Limit in position mode, the servo has no choice but to obey it,
 * since it directly sets the applied PWM duty rather than a PID cap. */
void STS_WritePWM(uint8_t id, int16_t duty_permille);

#endif /* FEETECH_STS_H */
