#ifndef _MDL_LIFT_CONFIG_H_
#define _MDL_LIFT_CONFIG_H_

#include <stdint.h>

/* Mechanical constants: measured or mechanically defined. */
/*
 * Default z used before homing has established a calibrated position.
 * This is not the final HOME position. HOME completion calibrates the lift
 * to the middle sensor edge position depending on the detected sensor edge.
 */
#define LIFT_Z_UNHOMED_DEFAULT_M (0.0f)
#define LIFT_Z_MIN_M (0.0f)
#define LIFT_Z_MAX_M (0.50f)

#define LIFT_TRAVEL_PER_ROUND_M (0.04f)
#define LIFT_TRAVEL_CALIBRATION_SCALE (1.0f)
#define LIFT_EFFECTIVE_TRAVEL_PER_ROUND_M \
    (LIFT_TRAVEL_PER_ROUND_M * LIFT_TRAVEL_CALIBRATION_SCALE)
#define LIFT_ENCODER_COUNT_PER_ROUND (1584U)

#define LIFT_M_PER_COUNT \
    (LIFT_EFFECTIVE_TRAVEL_PER_ROUND_M / (float)LIFT_ENCODER_COUNT_PER_ROUND)
#define LIFT_RPM_PER_MPS (60.0f / LIFT_EFFECTIVE_TRAVEL_PER_ROUND_M)

/* Positive motor rpm and encoder count both move the lift downward. */
#define LIFT_ENCODER_TO_Z_SIGN (-1.0f)
#define LIFT_V_TO_RPM_SIGN (-1.0f)

/* Control parameters: conservative initial values, tune by test. */
#define LIFT_MAX_V_MPS (0.08f)
#define LIFT_MIN_V_MPS (0.003f)
#define LIFT_REACHED_EPS_M (0.002f)
#define LIFT_POSITION_KP (7.0f)
#define LIFT_POSITION_KI (0.f)
#define LIFT_POSITION_KD (0.f)
#define LIFT_POSITION_INTEGRAL_LIMIT_M (0.10f)

#define LIFT_HOME_V_MPS (LIFT_MAX_V_MPS)
#define LIFT_HOME_MAX_TRAVEL_M (0.45f)
#define LIFT_HOME_TIMEOUT_MARGIN_MS (5000U)
#define LIFT_HOME_TIMEOUT_MS                                            \
    ((uint32_t)((LIFT_HOME_MAX_TRAVEL_M / LIFT_HOME_V_MPS) * 1000.0f) + \
     LIFT_HOME_TIMEOUT_MARGIN_MS)
#define LIFT_HOME_SENSOR_BLOCKED_LEVEL (0U)
#define LIFT_HOME_SENSOR_CLEAR_LEVEL (1U)
/*
 * HOME uses the middle sensor edge as the calibrated position. These values
 * are measured from the bottom reference plane. After HOME succeeds, current_z
 * and target_z are set to one of these values, not to
 * LIFT_Z_UNHOMED_DEFAULT_M.
 */
#define LIFT_HOME_UP_RISING_Z_M (LIFT_MIDDLE_UP_RISING_Z_M)
#define LIFT_HOME_UP_FALLING_Z_M (LIFT_MIDDLE_UP_FALLING_Z_M)
#define LIFT_HOME_DOWN_RISING_Z_M (LIFT_MIDDLE_DOWN_RISING_Z_M)
#define LIFT_HOME_DOWN_FALLING_Z_M (LIFT_MIDDLE_DOWN_FALLING_Z_M)

/* Reference sensor points measured from the bottom reference plane. */
#define LIFT_SENSOR_REF_MAX_ERROR_M (0.030f)
#define LIFT_REF_EDGE_LOCKOUT_MS (300U)

#define LIFT_MIDDLE_UP_RISING_Z_M (0.341f)
#define LIFT_MIDDLE_UP_FALLING_Z_M (0.246f)
#define LIFT_MIDDLE_DOWN_RISING_Z_M (0.246f)
#define LIFT_MIDDLE_DOWN_FALLING_Z_M (0.337f)

/* Top sensor reference points are physical calibration values, independent of
 * the normal MOVE_Z software limit. */
#define LIFT_TOP_UP_FALLING_Z_M (0.493f)
#define LIFT_TOP_DOWN_RISING_Z_M (0.490f)

#endif /* _MDL_LIFT_CONFIG_H_ */
