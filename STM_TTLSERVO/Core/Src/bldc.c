#include "bldc.h"
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim3;

/* =========================================================================
 * Q15 sine table  (256 entries, 0° … 358.6°)
 * Value = round(32767 × sin(i × 2π / 256))
 * ========================================================================= */
static const int16_t sin_q15[256] = {
       0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
    6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
   12539, 13279, 14010, 14733, 15446, 16151, 16846, 17530,
   18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
   23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
   27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
   30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
   32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
   32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
   32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
   30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683,
   27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
   23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
   18204, 17530, 16846, 16151, 15446, 14733, 14010, 13279,
   12539, 11793, 11039, 10278,  9512,  8739,  7962,  7179,
    6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,
       0,  -804, -1608, -2410, -3212, -4011, -4808, -5602,
   -6393, -7179, -7962, -8739, -9512,-10278,-11039,-11793,
  -12539,-13279,-14010,-14733,-15446,-16151,-16846,-17530,
  -18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
  -23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,
  -27245,-27683,-28105,-28510,-28898,-29268,-29621,-29956,
  -30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,
  -32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,
  -32767,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
  -32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
  -30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,
  -27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,
  -23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,
  -18204,-17530,-16846,-16151,-15446,-14733,-14010,-13279,
  -12539,-11793,-11039,-10278, -9512, -8739, -7962, -7179,
   -6393, -5602, -4808, -4011, -3212, -2410, -1608,  -804,
};

/* CCR midpoint = 50% duty.  All three phases are centered here. */
#define HALF_PERIOD  ((BLDC_PWM_PERIOD + 1u) / 2u)   /* 2000 */

/* Sinusoidal amplitude: CCR swings HALF_PERIOD ± s_duty. */
static uint32_t s_duty = 0U;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/* Update all three phase CCRs for electrical angle 'angle' (0–255 = 0°–360°).
 * Phase offsets: A=0°, B=−120° (table+171), C=+120° (table+85). */
static void commutate_angle(uint8_t angle)
{
    int32_t sa = sin_q15[angle];
    int32_t sb = sin_q15[(uint8_t)(angle + 171u)];
    int32_t sc = sin_q15[(uint8_t)(angle +  85u)];

    uint32_t ca = (uint32_t)(HALF_PERIOD + (int32_t)s_duty * sa / 32767);
    uint32_t cb = (uint32_t)(HALF_PERIOD + (int32_t)s_duty * sb / 32767);
    uint32_t cc = (uint32_t)(HALF_PERIOD + (int32_t)s_duty * sc / 32767);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ca);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cb);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, cc);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void BLDC_Init(void)
{
    BLDC_Stop();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(BLDC_SD1_PORT, BLDC_SD1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLDC_SD2_PORT, BLDC_SD2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLDC_SD3_PORT, BLDC_SD3_PIN, GPIO_PIN_SET);
    HAL_Delay(5);   /* lower FETs on (CCR=0) → bootstrap caps charge */
}

void BLDC_Stop(void)
{
    HAL_GPIO_WritePin(BLDC_SD1_PORT, BLDC_SD1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_SD2_PORT, BLDC_SD2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_SD3_PORT, BLDC_SD3_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    s_duty = 0U;
}

void BLDC_SetDuty(uint32_t duty)
{
    if (duty < BLDC_PWM_DUTY_MIN) duty = BLDC_PWM_DUTY_MIN;
    if (duty > BLDC_PWM_DUTY_MAX) duty = BLDC_PWM_DUTY_MAX;
    s_duty = duty;
}

/* Maps a discrete step (0–5) to the equivalent sinusoidal angle. */
void BLDC_CommutateStep(uint8_t step)
{
    commutate_angle((uint8_t)((step % BLDC_STEPS_PER_CYCLE) * (256u / BLDC_STEPS_PER_CYCLE)));
}

void BLDC_Run(uint32_t step_period_us, uint32_t num_cycles)
{
    /* Advance angle in Q8.8 fixed-point: 65536 = one full electrical cycle.
     * angle_inc = 65536 × 1000µs / full_cycle_us  (advance per 1ms tick). */
    uint32_t full_cycle_us = (uint32_t)step_period_us * BLDC_STEPS_PER_CYCLE;
    uint32_t angle_inc     = 65536000u / full_cycle_us;

    uint32_t angle_acc   = 0u;
    uint32_t cycles_done = 0u;

    while (num_cycles == 0u || cycles_done < num_cycles) {
        commutate_angle((uint8_t)(angle_acc >> 8));
        HAL_Delay(1);
        angle_acc += angle_inc;
        while (angle_acc >= 65536u) {
            angle_acc -= 65536u;
            cycles_done++;
        }
    }
}

void BLDC_StartupRamp(void)
{
    uint32_t period_us  = BLDC_STARTUP_STEP_PERIOD_US;
    uint32_t target_us  = BLDC_RPMToStepPeriodUs(BLDC_TARGET_RPM);
    uint32_t period_dec = (period_us > target_us)
                          ? (period_us - target_us) / BLDC_RAMP_CYCLES
                          : 0U;

    /* Full startup torque is applied immediately; only the electrical
     * frequency is ramped. Open-loop pole-slip risk comes from accelerating
     * faster than the (possibly geared) rotor can follow, not from applying
     * more amplitude — extra amplitude only adds torque margin. Ramping the
     * amplitude in lockstep with the frequency (as before) left the torque
     * unusable for most of the ramp, since it only neared BLDC_STARTUP_DUTY
     * on the final cycle. BLDC_STARTUP_DUTY is already sized to keep inrush
     * current in check, so applying it from the start is safe. */
    BLDC_SetDuty(BLDC_STARTUP_DUTY);

    for (uint32_t cycle = 0U; cycle < BLDC_RAMP_CYCLES; cycle++) {
        uint32_t full_cycle_us = period_us * BLDC_STEPS_PER_CYCLE;
        uint32_t angle_inc     = 65536000u / full_cycle_us;
        uint32_t angle_acc     = 0u;

        while (angle_acc < 65536u) {
            commutate_angle((uint8_t)(angle_acc >> 8));
            HAL_Delay(1);
            angle_acc += angle_inc;
        }

        if (period_us > target_us + period_dec) period_us -= period_dec;
        else                                     period_us  = target_us;
    }
}

uint32_t BLDC_RPMToStepPeriodUs(uint32_t rpm)
{
    if (rpm == 0U) return 0xFFFFFFFFU;
    return 60000000UL / ((uint32_t)rpm * MOTOR_POLE_PAIRS * BLDC_STEPS_PER_CYCLE);
}

void BLDC_Commutate(uint8_t elec_angle, uint32_t duty)
{
    if (duty > BLDC_PWM_DUTY_MAX) duty = BLDC_PWM_DUTY_MAX;
    s_duty = duty;
    commutate_angle(elec_angle);
}
