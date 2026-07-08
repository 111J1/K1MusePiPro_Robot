#include "dev_brush_motor.h"
#include <math.h>
#include "dev_brush_motor_config.h"

#define motor_constrain(x, min, max) ((x > max) ? max : (x < min ? min : x))

void brush_motor_init(void *ctx)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    if (motor != 0) {
        const brush_motor_driver_t *driver = motor->driver;

        if (motor->block_max_cnt == 0U) {
            motor->block_max_cnt = MOTOR_BLOCK_MAX_CNT_DEFAULT;
        }

        // work only when all driver functions are provided
        if (driver && driver->init && driver->set_duty && driver->set_direction &&
            driver->get_encoder_count && driver->get_duty) {
            motor->is_initialized = driver->init(driver->ctx);
        }
    }
}

void brush_motor_set_rpm(void *ctx, float rpm)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    if (motor != 0) {
        rpm = motor_constrain(rpm, -motor->max_rpm, motor->max_rpm);
        if (rpm < RPM_DEADZONE && rpm > -RPM_DEADZONE) { // dead zone
            rpm = 0.f;
        }
        motor->target_rpm = rpm;
    }
}

static void brush_motor_stop(brush_motor_t *motor)
{
    const brush_motor_driver_t *driver = motor->driver;

    incremental_pid_clear_out(&motor->pid);
    motor->target_direction = 0;

    driver->set_direction(driver->ctx, 0);
    driver->set_duty(driver->ctx, 0.f);
}

float brush_motor_get_rpm(void *ctx)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    if (motor != 0) {
        return motor->current_rpm;
    } else {
        return 0.f;
    }
}

int32_t brush_motor_get_total_encoder_count(void *ctx)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    return (motor != 0) ? motor->total_encoder_count : 0;
}

void brush_motor_reset_total_encoder_count(void *ctx, int32_t count)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    if (motor != 0) {
        motor->total_encoder_count = count;
    }
}

/* Note
 * dt(second), encoder_count is the count since last call
 */
void brush_motor_update(void *ctx, float dt)
{
    brush_motor_t *motor = (brush_motor_t *)ctx;

    if ((motor != 0) && (motor->is_initialized == 1)) {
        const brush_motor_driver_t *driver = motor->driver;

        // get encoder count
        int16_t encoder_count = driver->get_encoder_count(driver->ctx);
        motor->total_encoder_count += encoder_count;

        // update current_rpm and current_direction
        float current_rpm_raw =
            ((float)encoder_count / motor->one_round_encoder_count) / dt * 60.0f; // rpm
        motor->current_rpm =
            motor->current_rpm + RPM_FILTER_ALPHA * (current_rpm_raw - motor->current_rpm);

        // update current_direction
        if (motor->current_rpm >= 0.f) {
            motor->current_direction = 1;
        } else if (motor->current_rpm < 0.f) {
            motor->current_direction = -1;
        }

        uint8_t should_output = 1U;
        if (motor->is_blocked == 1U) {
            brush_motor_stop(motor);
            should_output = 0U;
            // block auto recovery
            if (++motor->blocked_cnt >= MOTOR_BLOCK_AUTO_CLEAR_CNT) {
                motor->blocked_cnt = 0U;
                motor->is_blocked = 0U;
            }
        } else {
            // update pid controller
            motor->pid.target = motor->target_rpm;
            motor->pid.measure = motor->current_rpm;
            incremental_pid_ctrl(&motor->pid);

            // block detection
            float abs_target_rpm = fabsf(motor->target_rpm);
            float abs_current_rpm = fabsf(motor->current_rpm);
            float abs_pid_out = fabsf(motor->pid.out);
            // block condition: high/low target
            uint8_t is_block_now =
                (abs_pid_out >= MOTOR_BLOCK_DUTY_THRESHOLD) &&
                (((abs_target_rpm >= MOTOR_BLOCK_TARGET_RPM_THRESHOLD) &&
                  (abs_current_rpm <= MOTOR_BLOCK_CURRENT_RPM_THRESHOLD)) ||
                 ((abs_target_rpm <= RPM_DEADZONE) &&
                  (abs_pid_out >= MOTOR_BLOCK_DUTY_THRESHOLD)));

            if (is_block_now) {
                if (++motor->blocked_cnt >= motor->block_max_cnt) {
                    motor->blocked_cnt = 0U;
                    motor->is_blocked = 1U;
                    brush_motor_stop(motor);
                    should_output = 0U;
                }
            } else {
                motor->blocked_cnt = 0U;
            }
        }

        if (should_output == 1U) {
            if (motor->pid.out >= 0.f) {
                motor->target_direction = 1;
            } else if (motor->pid.out < 0.f) {
                motor->target_direction = -1;
            }

            // output to driver
            driver->set_direction(driver->ctx, motor->target_direction);
            driver->set_duty(driver->ctx, fabsf(motor->pid.out));
        }
    }
}
