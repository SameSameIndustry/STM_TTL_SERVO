#ifndef INC_BLDC_H_
#define INC_BLDC_H_

#include "main.h"
#include <stdint.h>

/* =============================================================================
 * Motor Parameters
 * Adjust these values to match your BLDC motor specifications.
 * See README.md for detailed explanation of each parameter.
 * ============================================================================= */

/** KV rating (no-load RPM per Volt) */
#define MOTOR_KV                    100

/** Number of pole pairs  (total poles / 2).  14P motor = 7 pairs */
#define MOTOR_POLE_PAIRS            7

/** Phase resistance in milliohms  (11.2 Ω = 11200 mΩ) */
#define MOTOR_RESISTANCE_MOHM       11200

/** Actual supply voltage in millivolts (battery / power supply) */
#define MOTOR_SUPPLY_VOLTAGE_MV     8000   /* 11.1 V (3S LiPo) */

/** Maximum rated voltage in millivolts */
#define MOTOR_MAX_VOLTAGE_MV        14800

/** Maximum phase current in milliamps */
#define MOTOR_MAX_CURRENT_MA        5000

/** Maximum mechanical RPM */
#define MOTOR_MAX_RPM               2180

/* =============================================================================
 * PWM Configuration
 * Must match TIM3 settings configured in CubeMX.
 * ============================================================================= */

/** TIM3 Auto-Reload Register value  (ARR).
 *  PWM frequency = SYSCLK / (PSC+1) / (ARR+1) = 80MHz / 1 / 4000 = 20 kHz */
#define BLDC_PWM_PERIOD             3999U

/** Minimum sinusoidal amplitude (CCR swings ±200 around 2000 = 45–55% duty). */
#define BLDC_PWM_DUTY_MIN           ((BLDC_PWM_PERIOD + 1) / 20)  /* 5% amp = 200 */

/** Maximum sinusoidal amplitude (CCR swings ±1800 around 2000 = 5–95% duty).
 *  Capped at 90% of half-period so the bootstrap cap always recharges
 *  (minimum lower-FET on-time = 5% = 2.5 µs >> τ_bootstrap ≈ 0.4 µs). */
#define BLDC_PWM_DUTY_MAX           ((BLDC_PWM_PERIOD + 1) * 9 / 20)  /* 90% amp = 1800 */

/* =============================================================================
 * Startup / Ramp Parameters
 * ============================================================================= */

/** Phase voltage to apply during startup [mV].
 *  Determines how hard the motor is driven at startup.
 *  Keep low to limit inrush current; raise if the motor fails to start. */
#define BLDC_STARTUP_VOLTAGE_MV     7200U

/** Sinusoidal amplitude at end of startup ramp.
 *  amplitude = (V_startup / V_supply) × half_period */
#define BLDC_STARTUP_DUTY           ((BLDC_PWM_PERIOD + 1) / 2 \
                                     * BLDC_STARTUP_VOLTAGE_MV \
                                     / MOTOR_SUPPLY_VOLTAGE_MV)

/** Initial commutation step period [µs] at the beginning of the ramp.
 *  Corresponds to approximately 70 RPM for a 7 pole-pair motor. */
#define BLDC_STARTUP_STEP_PERIOD_US 100000U

/** Target mechanical RPM at the end of the startup ramp. */
#define BLDC_TARGET_RPM             700U

/** Number of full electrical cycles used for the acceleration ramp. */
#define BLDC_RAMP_CYCLES            40U

/* =============================================================================
 * Closed-Loop Position Control (velocity cascade)
 * Sensorless-style open-loop commutation (a free-running field angle, same
 * mechanism as BLDC_StartupRamp/BLDC_Run, at a fixed proven-safe duty) whose
 * rotation SPEED — not its duty/torque — is closed-loop controlled from the
 * AS5600 position error. This avoids the too-weak torque a duty-proportional
 * -to-error scheme produces for small errors: duty always stays at a level
 * already known to move this (geared) load.
 * ============================================================================= */

/** AS5600 encoder resolution (12-bit → counts per mechanical revolution). */
#define BLDC_ENCODER_COUNTS         4096U

/** Duty used to hold the rotor while settling at a known starting field
 *  angle, and during direction calibration, below. */
#define BLDC_ALIGN_DUTY             BLDC_STARTUP_DUTY

/** Time to let the rotor settle during alignment / direction calibration
 *  [ms]. */
#define BLDC_ALIGN_TIME_MS          500U

/** Electrical-angle step (0-255 scale) used right after settling to measure
 *  which way the encoder count moves for increasing electrical angle. The
 *  velocity-cascade loop below only drives toward the target if that
 *  correspondence is known — it depends on motor winding / encoder
 *  mounting, which is not otherwise known, so it is measured instead of
 *  assumed. 32 = 45° electrical. */
#define BLDC_DIR_CAL_ELEC_STEP      32u

/** Position control loop period [ms] (also the velocity loop's dt). */
#define BLDC_POS_LOOP_PERIOD_MS     1U

/** Fixed duty applied while the position loop is actively driving the
 *  field — reuses the proven-safe BLDC_STARTUP_DUTY rather than scaling
 *  duty with position error. */
#define BLDC_POS_RUN_DUTY           BLDC_STARTUP_DUTY

/** Maximum commanded angular speed [°/s mechanical] for the position loop —
 *  the fastest continuous rotation this motor/gearbox/duty combination has
 *  actually been confirmed to sustain: BLDC_TARGET_RPM, the speed
 *  BLDC_StartupRamp() ramps up to and runs at. Deliberately NOT tied to
 *  BLDC_POS_MAX_ACCEL_DEG_S2 below — raising the cruise-speed ceiling must
 *  not silently raise acceleration too (that previously caused stalls).
 *  Reaching this speed from a standstill is only safe because the
 *  acceleration limit ramps up to it gradually instead of jumping straight
 *  there. */
#define BLDC_POS_MAX_SPEED_DEG_S    ((float)BLDC_TARGET_RPM * 6.0f)

/** Maximum acceleration [°/s²] for the position loop's speed command. Kept
 *  as a plain, independently-set value — NOT derived from
 *  BLDC_POS_MAX_SPEED_DEG_S / a time constant, since that coupling meant
 *  raising the speed ceiling silently made acceleration more aggressive
 *  too. ~214 was confirmed to accelerate cleanly without the rotor losing
 *  sync; this is 1.5x that as a first step up. Raise further in small
 *  increments; lower it if the shaft loses sync (no motion / juddering)
 *  when starting a move — a longer time to reach BLDC_POS_MAX_SPEED_DEG_S
 *  is the trade-off for a lower value. */
#define BLDC_POS_MAX_ACCEL_DEG_S2   3000.0f

/* =============================================================================
 * Derived constants (do not edit)
 * ============================================================================= */
#define BLDC_STEPS_PER_CYCLE        6U
#define BLDC_STEPS_PER_REV          (BLDC_STEPS_PER_CYCLE * MOTOR_POLE_PAIRS)

/* =============================================================================
 * GPIO Assignments  (from schematic / netlist)
 * ============================================================================= */
#define BLDC_SD1_PORT   GPIOA
#define BLDC_SD1_PIN    GPIO_PIN_0   /* PA0 → EG2104 U4 SD  (Phase A) */
#define BLDC_SD2_PORT   GPIOA
#define BLDC_SD2_PIN    GPIO_PIN_1   /* PA1 → EG2104 U5 SD  (Phase B) */
#define BLDC_SD3_PORT   GPIOA
#define BLDC_SD3_PIN    GPIO_PIN_2   /* PA2 → EG2104 U6 SD  (Phase C) */

/* =============================================================================
 * Public API
 * ============================================================================= */

/**
 * @brief  Initialise the BLDC driver.
 *         Must be called after MX_TIM3_Init().
 *         Starts all three PWM channels and enables DWT for µs timing.
 */
void BLDC_Init(void);

/**
 * @brief  Immediately disable all gate drivers (safe state).
 */
void BLDC_Stop(void);

/**
 * @brief  Set PWM duty cycle applied to the active high-side phase.
 * @param  duty  0 (0 %) … BLDC_PWM_PERIOD (100 %)
 */
void BLDC_SetDuty(uint32_t duty);

/**
 * @brief  Apply a single 6-step commutation state.
 * @param  step  0 … 5
 */
void BLDC_CommutateStep(uint8_t step);

/**
 * @brief  Run forced commutation continuously at a fixed step period.
 * @param  step_period_us  Time between commutation steps [µs]
 * @param  num_cycles      Electrical cycles to execute (0 = run forever)
 */
void BLDC_Run(uint32_t step_period_us, uint32_t num_cycles);

/**
 * @brief  Ramp from standstill to BLDC_TARGET_RPM using open-loop commutation.
 */
void BLDC_StartupRamp(void);

/**
 * @brief  Convert mechanical RPM to commutation step period in microseconds.
 * @param  rpm  Mechanical RPM
 * @return Step period [µs]
 */
uint32_t BLDC_RPMToStepPeriodUs(uint32_t rpm);

/**
 * @brief  Directly command the stator field at an absolute electrical angle.
 *         Called once per control cycle by the closed-loop position
 *         controller with the angle/duty computed from encoder feedback.
 * @param  elec_angle  0-255 = 0-360° electrical.
 * @param  duty        Sinusoidal amplitude, 0 (no torque) ... BLDC_PWM_DUTY_MAX.
 */
void BLDC_Commutate(uint8_t elec_angle, uint32_t duty);

#endif /* INC_BLDC_H_ */
