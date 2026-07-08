#ifndef _MDL_CHASSIS_H_
#define _MDL_CHASSIS_H_

#include "dev_brush_motor.h"

typedef enum {
    MOTOR_LF = 0,
    MOTOR_RF,
    MOTOR_RB,
    MOTOR_LB,
    MOTOR_COUNT,
} motor_index_e;

typedef enum {
    CHASSIS_MOVE_MODE_LCS = 0, // local coordinate system
    CHASSIS_MOVE_MODE_WCS,     // world coordinate system
} chassis_move_CS_e;

typedef struct chassis_motor_class {
    void *ctx; // user defined context
    void (*init)(void *ctx);
    void (*set_rpm)(void *ctx, float rpm);
    float (*get_rpm)(void *ctx);
    void (*update)(void *ctx, float dt);
} chassis_motor_t;

typedef struct chassis_class {
    // information
    float current_V;
    float target_V;
    float current_omega; // rad/s
    float target_omega;
    float current_WCS_direction; // in radians, 0 is forward, positive is counter-clockwise, and in WCS
    float target_WCS_direction;
    float current_WCS_Vx; // in m/s, in WCS
    float current_WCS_Vy;
    float current_WCS_X; // in meters, in WCS
    float current_WCS_Y;
    float current_LCS_direction; // in radians, 0 is forward, positive is counter-clockwise, and in LCS
    float target_LCS_direction;

    // mode
    chassis_move_CS_e move_CS;

    // motor instances
    chassis_motor_t motor[MOTOR_COUNT];
} chassis_t;

void chassis_init(chassis_t *chassis);
void chassis_set_movement(chassis_t *chassis, chassis_move_CS_e move_CS,
                          float direction, float v, float omega);
void chassis_reset_WCS_and_odometry(chassis_t *chassis, float WCS_direction,
                                    float WCS_X, float WCS_Y);
void chassis_update(chassis_t *chassis, float dt);

#endif /* _MDL_CHASSIS_H_ */
