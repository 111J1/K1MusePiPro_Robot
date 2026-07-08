#ifndef _MDL_LIFT_H_
#define _MDL_LIFT_H_

#include <stdint.h>
#include "pid.h"

typedef enum {
    LIFT_STATE_IDLE = 0,
    LIFT_STATE_HOMING,
    LIFT_STATE_MOVING,
    LIFT_STATE_REACHED,
    LIFT_STATE_FAULT,
} lift_state_e;

typedef enum {
    LIFT_HOME_STATE_IDLE = 0,
    LIFT_HOME_STATE_WAIT_RISING_EDGE,
    LIFT_HOME_STATE_WAIT_FALLING_EDGE,
    LIFT_HOME_STATE_DONE,
    LIFT_HOME_STATE_FAULT,
} lift_home_state_e;

typedef enum {
    LIFT_REF_SENSOR_NONE = 0,
    LIFT_REF_SENSOR_MIDDLE,
    LIFT_REF_SENSOR_TOP,
} lift_ref_sensor_e;

typedef enum {
    LIFT_REF_EDGE_NONE = 0,
    LIFT_REF_EDGE_RISING,
    LIFT_REF_EDGE_FALLING,
} lift_ref_edge_e;

typedef enum {
    LIFT_REF_RESULT_IGNORED = 0,
    LIFT_REF_RESULT_APPLIED,
    LIFT_REF_RESULT_MISMATCH,
} lift_ref_result_e;

typedef struct lift_motor_class {
    void *ctx;
    void (*init)(void *ctx);
    void (*set_rpm)(void *ctx, float rpm);
    float (*get_rpm)(void *ctx);
    int32_t (*get_total_encoder_count)(void *ctx);
    void (*reset_total_encoder_count)(void *ctx, int32_t count);
    void (*update)(void *ctx, float dt);
} lift_motor_t;

typedef struct lift_home_sensor_class {
    void *ctx;
    void (*init)(void *ctx);
    void (*update)(void *ctx);
    uint8_t (*get_level)(void *ctx);
    uint8_t (*get_last_level)(void *ctx);
    uint8_t (*get_rising_edge)(void *ctx);
    uint8_t (*get_falling_edge)(void *ctx);
    uint8_t (*is_initialized)(void *ctx);
} lift_home_sensor_t;

typedef struct lift_class {
    float current_z;
    float target_z;
    float current_v;
    float target_v;

    float z_min;
    float z_max;
    float max_v;
    float reached_eps;
    float position_kp;
    position_pid_ctrl_t position_pid;
    uint8_t is_homed;
    uint8_t home_start_level;
    lift_home_state_e home_state;

    lift_ref_sensor_e last_ref_sensor;
    lift_ref_edge_e last_ref_edge;
    float last_ref_z;
    float last_ref_error;
    uint32_t middle_last_ref_ms;
    uint32_t top_last_ref_ms;

    lift_state_e state;
    lift_motor_t motor;
    lift_home_sensor_t home_sensor;
} lift_t;

void lift_init(lift_t *lift);
uint8_t lift_target_z_is_valid(const lift_t *lift, float z);
uint8_t lift_set_target_z(lift_t *lift, float z);
void lift_set_velocity(lift_t *lift, float v);
void lift_start_home(lift_t *lift);
void lift_stop(lift_t *lift);
void lift_calibrate_position(lift_t *lift, float z);
void lift_clear_reference_state(lift_t *lift);
lift_ref_result_e lift_process_reference_event(lift_t *lift,
                                               lift_ref_sensor_e sensor,
                                               lift_ref_edge_e edge,
                                               uint32_t now_ms);
void lift_update(lift_t *lift, float dt);
uint8_t lift_is_reached(const lift_t *lift);
uint8_t lift_is_homed(const lift_t *lift);
uint8_t lift_is_home_sensor_initialized(const lift_t *lift);
uint8_t lift_get_home_sensor_level(const lift_t *lift);
uint8_t lift_get_home_sensor_last_level(const lift_t *lift);
uint8_t lift_get_home_sensor_rising_edge(const lift_t *lift);
uint8_t lift_get_home_sensor_falling_edge(const lift_t *lift);

#endif /* _MDL_LIFT_H_ */
