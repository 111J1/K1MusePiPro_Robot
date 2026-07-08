#ifndef _DEV_BRUSH_MOTOR_H_
#define _DEV_BRUSH_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct brush_motor_driver_class {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    void (*set_duty)(const void *ctx, float duty);
    void (*set_direction)(const void *ctx, int8_t direction);
    float (*get_duty)(const void *ctx);
    int16_t (*get_encoder_count)(const void *ctx);
} brush_motor_driver_t;

/* Note
 * rpm: 0-loaded_max_rpm(rpm), after reduction
 * direction: -1: reverse, 0: stop, 1: forward
 */
typedef struct brush_motor_class {
    // information
    float current_rpm;
    float target_rpm;
    int8_t current_direction;
    int8_t target_direction;
    int32_t total_encoder_count;
    uint16_t blocked_cnt;

    // state
    uint8_t is_initialized;
    uint8_t is_blocked;

    // pid controller
    incremental_pid_ctrl_t pid;

    // parameters
    float max_rpm;
    uint16_t block_max_cnt;
    uint16_t one_round_encoder_count; // calculate by yourself

    // driver interface, must be provided by user
    const brush_motor_driver_t *driver;
} brush_motor_t;

void brush_motor_init(void *ctx);
void brush_motor_set_rpm(void *ctx, float rpm);
float brush_motor_get_rpm(void *ctx);
int32_t brush_motor_get_total_encoder_count(void *ctx);
void brush_motor_reset_total_encoder_count(void *ctx, int32_t count);
void brush_motor_update(void *ctx, float dt);

#endif /* _DEV_BRUSH_MOTOR_H_ */
