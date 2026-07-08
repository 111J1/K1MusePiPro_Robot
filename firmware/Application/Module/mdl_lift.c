#include "mdl_lift.h"

#include "mdl_lift_config.h"
#include <math.h>

static float lift_constrain(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

static float lift_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

typedef enum {
    LIFT_CALIB_POINT_HOME_RISING = 0,
    LIFT_CALIB_POINT_HOME_FALLING,
} lift_calib_point_e;

static int32_t lift_z_to_encoder_count(float z)
{
    float count = z / (LIFT_M_PER_COUNT * LIFT_ENCODER_TO_Z_SIGN);

    return (count >= 0.0f) ? (int32_t)(count + 0.5f) : (int32_t)(count - 0.5f);
}

static float lift_get_calib_z(lift_calib_point_e point, float v)
{
    switch (point) {
    case LIFT_CALIB_POINT_HOME_RISING:
        if (v > 0.0f) {
            return LIFT_HOME_UP_RISING_Z_M;
        }
        if (v < 0.0f) {
            return LIFT_HOME_DOWN_RISING_Z_M;
        }
        return LIFT_HOME_UP_RISING_Z_M;

    case LIFT_CALIB_POINT_HOME_FALLING:
        if (v > 0.0f) {
            return LIFT_HOME_UP_FALLING_Z_M;
        }
        if (v < 0.0f) {
            return LIFT_HOME_DOWN_FALLING_Z_M;
        }
        return LIFT_HOME_UP_FALLING_Z_M;

    default:
        return LIFT_Z_UNHOMED_DEFAULT_M;
    }
}

static uint8_t lift_get_reference_z(const lift_t *lift,
                                    lift_ref_sensor_e sensor,
                                    lift_ref_edge_e edge,
                                    float *ref_z)
{
    if ((lift == 0) || (ref_z == 0)) {
        return 0U;
    }

    if (lift->target_v > 0.0f) {
        if ((sensor == LIFT_REF_SENSOR_MIDDLE) &&
            (edge == LIFT_REF_EDGE_RISING)) {
            *ref_z = LIFT_MIDDLE_UP_RISING_Z_M;
            return 1U;
        }
        if ((sensor == LIFT_REF_SENSOR_MIDDLE) &&
            (edge == LIFT_REF_EDGE_FALLING)) {
            *ref_z = LIFT_MIDDLE_UP_FALLING_Z_M;
            return 1U;
        }
        if ((sensor == LIFT_REF_SENSOR_TOP) &&
            (edge == LIFT_REF_EDGE_FALLING)) {
            *ref_z = LIFT_TOP_UP_FALLING_Z_M;
            return 1U;
        }
    } else if (lift->target_v < 0.0f) {
        if ((sensor == LIFT_REF_SENSOR_MIDDLE) &&
            (edge == LIFT_REF_EDGE_RISING)) {
            *ref_z = LIFT_MIDDLE_DOWN_RISING_Z_M;
            return 1U;
        }
        if ((sensor == LIFT_REF_SENSOR_MIDDLE) &&
            (edge == LIFT_REF_EDGE_FALLING)) {
            *ref_z = LIFT_MIDDLE_DOWN_FALLING_Z_M;
            return 1U;
        }
        if ((sensor == LIFT_REF_SENSOR_TOP) &&
            (edge == LIFT_REF_EDGE_RISING)) {
            *ref_z = LIFT_TOP_DOWN_RISING_Z_M;
            return 1U;
        }
    }

    return 0U;
}

static uint8_t lift_reference_edge_can_accept(lift_t *lift,
                                              lift_ref_sensor_e sensor,
                                              uint32_t now_ms)
{
    uint32_t *last_ms;

    if (sensor == LIFT_REF_SENSOR_MIDDLE) {
        last_ms = &lift->middle_last_ref_ms;
    } else if (sensor == LIFT_REF_SENSOR_TOP) {
        last_ms = &lift->top_last_ref_ms;
    } else {
        return 1U;
    }

    if ((*last_ms != 0U) &&
        ((uint32_t)(now_ms - *last_ms) < LIFT_REF_EDGE_LOCKOUT_MS)) {
        return 0U;
    }

    *last_ms = now_ms;
    return 1U;
}

static void lift_apply_defaults(lift_t *lift)
{
    if (lift->z_max == 0.0f) {
        lift->z_max = LIFT_Z_MAX_M;
    }
    lift->z_min = LIFT_Z_MIN_M;

    if (lift->max_v <= 0.0f) {
        lift->max_v = LIFT_MAX_V_MPS;
    }
    if (lift->reached_eps <= 0.0f) {
        lift->reached_eps = LIFT_REACHED_EPS_M;
    }
    if (lift->position_kp <= 0.0f) {
        lift->position_kp = LIFT_POSITION_KP;
    }

    lift->position_pid.kp = lift->position_kp;
    lift->position_pid.ki = LIFT_POSITION_KI;
    lift->position_pid.kd = LIFT_POSITION_KD;
    lift->position_pid.integral_min = -LIFT_POSITION_INTEGRAL_LIMIT_M;
    lift->position_pid.integral_max = LIFT_POSITION_INTEGRAL_LIMIT_M;
    lift->position_pid.out_min = -lift->max_v;
    lift->position_pid.out_max = lift->max_v;
}

void lift_init(lift_t *lift)
{
    if (lift == 0) {
        return;
    }

    lift_apply_defaults(lift);

    if (lift->motor.init != 0) {
        lift->motor.init(lift->motor.ctx);
    }
    if (lift->motor.reset_total_encoder_count != 0) {
        lift->motor.reset_total_encoder_count(lift->motor.ctx, 0);
    }
    if (lift->home_sensor.init != 0) {
        lift->home_sensor.init(lift->home_sensor.ctx);
    }

    lift->current_z = LIFT_Z_UNHOMED_DEFAULT_M;
    lift->target_z = LIFT_Z_UNHOMED_DEFAULT_M;
    lift->current_v = 0.0f;
    lift->target_v = 0.0f;
    lift->state = LIFT_STATE_IDLE;
    lift->is_homed = 0U;
    lift->home_start_level = LIFT_HOME_SENSOR_CLEAR_LEVEL;
    lift->home_state = LIFT_HOME_STATE_IDLE;
    lift_clear_reference_state(lift);
    position_pid_clear_out(&lift->position_pid);
}

uint8_t lift_target_z_is_valid(const lift_t *lift, float z)
{
    if (lift == 0) {
        return 0U;
    }

    return ((z >= lift->z_min) && (z <= lift->z_max)) ? 1U : 0U;
}

uint8_t lift_set_target_z(lift_t *lift, float z)
{
    if ((lift == 0) || (lift_target_z_is_valid(lift, z) == 0U)) {
        return 0U;
    }

    lift->target_z = z;
    position_pid_clear_out(&lift->position_pid);
    lift->state = LIFT_STATE_MOVING;
    return 1U;
}

void lift_start_home(lift_t *lift)
{
    uint8_t level;

    if (lift == 0) {
        return;
    }

    if ((lift->home_sensor.is_initialized == 0) ||
        (lift->home_sensor.is_initialized(lift->home_sensor.ctx) == 0U) ||
        (lift->home_sensor.get_level == 0)) {
        lift_stop(lift);
        lift->home_state = LIFT_HOME_STATE_FAULT;
        lift->state = LIFT_STATE_FAULT;
        return;
    }

    if (lift->home_sensor.update != 0) {
        lift->home_sensor.update(lift->home_sensor.ctx);
    }

    level = lift->home_sensor.get_level(lift->home_sensor.ctx);
    lift->is_homed = 0U;
    lift->home_start_level = level;
    position_pid_clear_out(&lift->position_pid);
    lift->state = LIFT_STATE_HOMING;

    if (level == LIFT_HOME_SENSOR_BLOCKED_LEVEL) {
        lift->home_state = LIFT_HOME_STATE_WAIT_RISING_EDGE;
        lift_set_velocity(lift, -LIFT_HOME_V_MPS);
    } else {
        lift->home_state = LIFT_HOME_STATE_WAIT_FALLING_EDGE;
        lift_set_velocity(lift, -LIFT_HOME_V_MPS);
    }
}

void lift_set_velocity(lift_t *lift, float v)
{
    float rpm;

    if (lift == 0) {
        return;
    }

    lift->target_v = lift_constrain(v, -lift->max_v, lift->max_v);
    rpm = lift->target_v * LIFT_RPM_PER_MPS * LIFT_V_TO_RPM_SIGN;

    if (lift->motor.set_rpm != 0) {
        lift->motor.set_rpm(lift->motor.ctx, rpm);
    }
}

void lift_stop(lift_t *lift)
{
    if (lift == 0) {
        return;
    }

    lift->target_v = 0.0f;
    position_pid_clear_out(&lift->position_pid);
    if (lift->motor.set_rpm != 0) {
        lift->motor.set_rpm(lift->motor.ctx, 0.0f);
    }
}

void lift_calibrate_position(lift_t *lift, float z)
{
    float constrained_z;

    if (lift == 0) {
        return;
    }

    constrained_z = lift_constrain(z, lift->z_min, lift->z_max);
    if (lift->motor.reset_total_encoder_count != 0) {
        lift->motor.reset_total_encoder_count(lift->motor.ctx,
                                              lift_z_to_encoder_count(constrained_z));
    }

    lift->current_z = constrained_z;
    position_pid_clear_out(&lift->position_pid);
}

void lift_clear_reference_state(lift_t *lift)
{
    if (lift == 0) {
        return;
    }

    lift->last_ref_sensor = LIFT_REF_SENSOR_NONE;
    lift->last_ref_edge = LIFT_REF_EDGE_NONE;
    lift->last_ref_z = 0.0f;
    lift->last_ref_error = 0.0f;
    lift->middle_last_ref_ms = 0U;
    lift->top_last_ref_ms = 0U;
}

lift_ref_result_e lift_process_reference_event(lift_t *lift,
                                               lift_ref_sensor_e sensor,
                                               lift_ref_edge_e edge,
                                               uint32_t now_ms)
{
    float ref_z = 0.0f;
    float error;

    if (lift == 0) {
        return LIFT_REF_RESULT_IGNORED;
    }
    if ((lift->is_homed == 0U) || (lift->state != LIFT_STATE_MOVING)) {
        return LIFT_REF_RESULT_IGNORED;
    }
    if (lift_get_reference_z(lift, sensor, edge, &ref_z) == 0U) {
        return LIFT_REF_RESULT_IGNORED;
    }
    if (lift_reference_edge_can_accept(lift, sensor, now_ms) == 0U) {
        return LIFT_REF_RESULT_IGNORED;
    }

    error = lift->current_z - ref_z;
    lift->last_ref_sensor = sensor;
    lift->last_ref_edge = edge;
    lift->last_ref_z = ref_z;
    lift->last_ref_error = error;

    if (lift_absf(error) > LIFT_SENSOR_REF_MAX_ERROR_M) {
        return LIFT_REF_RESULT_MISMATCH;
    }

    lift_calibrate_position(lift, ref_z);
    return LIFT_REF_RESULT_APPLIED;
}

static void lift_update_feedback(lift_t *lift)
{
    int32_t encoder_count = 0;
    float rpm = 0.0f;

    if (lift->motor.get_total_encoder_count != 0) {
        encoder_count = lift->motor.get_total_encoder_count(lift->motor.ctx);
    }
    if (lift->motor.get_rpm != 0) {
        rpm = lift->motor.get_rpm(lift->motor.ctx);
    }

    lift->current_z = (float)encoder_count * LIFT_M_PER_COUNT * LIFT_ENCODER_TO_Z_SIGN;
    lift->current_v = rpm * (LIFT_EFFECTIVE_TRAVEL_PER_ROUND_M / 60.0f) * LIFT_ENCODER_TO_Z_SIGN;
}

static void lift_update_home_sensor(lift_t *lift)
{
    if (lift->home_sensor.update != 0) {
        lift->home_sensor.update(lift->home_sensor.ctx);
    }
}

static void lift_apply_calib_point(lift_t *lift, lift_calib_point_e point,
                                   float v, uint8_t reset_target)
{
    float z = lift_get_calib_z(point, v);

    if (lift->motor.reset_total_encoder_count != 0) {
        lift->motor.reset_total_encoder_count(lift->motor.ctx,
                                              lift_z_to_encoder_count(z));
    }

    lift->current_z = z;
    if (reset_target != 0U) {
        lift->target_z = z;
    }
    lift->current_v = 0.0f;
    position_pid_clear_out(&lift->position_pid);
    lift->is_homed = 1U;
}

static void lift_finish_home(lift_t *lift, lift_calib_point_e point)
{
    float home_v = lift->target_v;

    lift_stop(lift);
    lift_apply_calib_point(lift, point, home_v, 1U);
    lift->target_v = 0.0f;
    lift->home_state = LIFT_HOME_STATE_DONE;
    lift->state = LIFT_STATE_REACHED;
}

static void lift_update_home_reference(lift_t *lift)
{
    uint8_t rising_edge = 0U;
    uint8_t falling_edge = 0U;

    if (lift->is_homed == 0U) {
        return;
    }
    if (lift->state == LIFT_STATE_HOMING) {
        return;
    }

    if (lift->home_sensor.get_rising_edge != 0) {
        rising_edge = lift->home_sensor.get_rising_edge(lift->home_sensor.ctx);
    }
    if (lift->home_sensor.get_falling_edge != 0) {
        falling_edge = lift->home_sensor.get_falling_edge(lift->home_sensor.ctx);
    }

    if (((lift->target_v > 0.0f) && (rising_edge != 0U)) ||
        ((lift->target_v < 0.0f) && (falling_edge != 0U))) {
        if (rising_edge != 0U) {
            lift_apply_calib_point(lift, LIFT_CALIB_POINT_HOME_RISING,
                                   lift->target_v, 0U);
        } else {
            lift_apply_calib_point(lift, LIFT_CALIB_POINT_HOME_FALLING,
                                   lift->target_v, 0U);
        }
    }
}

static void lift_update_home(lift_t *lift)
{
    uint8_t level = lift->home_start_level;
    uint8_t rising_edge = 0U;
    uint8_t falling_edge = 0U;

    if (lift->home_sensor.get_level != 0) {
        level = lift->home_sensor.get_level(lift->home_sensor.ctx);
    }
    if (lift->home_sensor.get_rising_edge != 0) {
        rising_edge = lift->home_sensor.get_rising_edge(lift->home_sensor.ctx);
    }
    if (lift->home_sensor.get_falling_edge != 0) {
        falling_edge = lift->home_sensor.get_falling_edge(lift->home_sensor.ctx);
    }

    switch (lift->home_state) {
    case LIFT_HOME_STATE_WAIT_RISING_EDGE:
        if ((rising_edge != 0U) || (level != lift->home_start_level)) {
            lift_finish_home(lift, LIFT_CALIB_POINT_HOME_RISING);
        }
        break;

    case LIFT_HOME_STATE_WAIT_FALLING_EDGE:
        if ((falling_edge != 0U) || (level != lift->home_start_level)) {
            lift_finish_home(lift, LIFT_CALIB_POINT_HOME_FALLING);
        }
        break;

    case LIFT_HOME_STATE_DONE:
    case LIFT_HOME_STATE_IDLE:
    case LIFT_HOME_STATE_FAULT:
    default:
        break;
    }
}

void lift_update(lift_t *lift, float dt)
{
    float error;
    float target_v;

    if (lift == 0) {
        return;
    }

    if (lift->motor.update != 0) {
        lift->motor.update(lift->motor.ctx, dt);
    }
    lift_update_home_sensor(lift);

    lift_update_feedback(lift);
    lift_update_home_reference(lift);

    if (lift->state == LIFT_STATE_HOMING) {
        lift_update_home(lift);
        return;
    }

    if (lift->state != LIFT_STATE_MOVING) {
        return;
    }

    error = lift->target_z - lift->current_z;
    if (lift_absf(error) <= lift->reached_eps) {
        lift_stop(lift);
        lift->state = LIFT_STATE_REACHED;
        return;
    }

    lift->position_pid.target = lift->target_z;
    lift->position_pid.measure = lift->current_z;
    position_pid_ctrl(&lift->position_pid);
    target_v = lift_constrain(lift->position_pid.out, -lift->max_v, lift->max_v);
    if ((lift_absf(target_v) < LIFT_MIN_V_MPS) &&
        (lift_absf(error) > lift->reached_eps)) {
        target_v = (error > 0.0f) ? LIFT_MIN_V_MPS : -LIFT_MIN_V_MPS;
    }

    lift_set_velocity(lift, target_v);
}

uint8_t lift_is_reached(const lift_t *lift)
{
    return ((lift != 0) && (lift->state == LIFT_STATE_REACHED)) ? 1U : 0U;
}

uint8_t lift_is_homed(const lift_t *lift)
{
    return ((lift != 0) && (lift->is_homed != 0U)) ? 1U : 0U;
}

uint8_t lift_is_home_sensor_initialized(const lift_t *lift)
{
    if ((lift == 0) || (lift->home_sensor.is_initialized == 0)) {
        return 0U;
    }

    return lift->home_sensor.is_initialized(lift->home_sensor.ctx);
}

uint8_t lift_get_home_sensor_level(const lift_t *lift)
{
    if ((lift == 0) || (lift->home_sensor.get_level == 0)) {
        return 0U;
    }

    return lift->home_sensor.get_level(lift->home_sensor.ctx);
}

uint8_t lift_get_home_sensor_last_level(const lift_t *lift)
{
    if ((lift == 0) || (lift->home_sensor.get_last_level == 0)) {
        return 0U;
    }

    return lift->home_sensor.get_last_level(lift->home_sensor.ctx);
}

uint8_t lift_get_home_sensor_rising_edge(const lift_t *lift)
{
    if ((lift == 0) || (lift->home_sensor.get_rising_edge == 0)) {
        return 0U;
    }

    return lift->home_sensor.get_rising_edge(lift->home_sensor.ctx);
}

uint8_t lift_get_home_sensor_falling_edge(const lift_t *lift)
{
    if ((lift == 0) || (lift->home_sensor.get_falling_edge == 0)) {
        return 0U;
    }

    return lift->home_sensor.get_falling_edge(lift->home_sensor.ctx);
}
