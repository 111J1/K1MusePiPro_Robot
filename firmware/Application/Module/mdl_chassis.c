#include "mdl_chassis.h"
#include "mdl_chassis_config.h"
#include <math.h>

void chassis_init(chassis_t *chassis)
{
    if (chassis != 0) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (chassis->motor[i].init != 0) {
                chassis->motor[i].init(chassis->motor[i].ctx);
            }
        }
    }
}

// V is absolute value, direction is independent, and omega is independent
void chassis_set_movement(chassis_t *chassis, chassis_move_CS_e move_CS,
                          float direction, float V, float omega)
{
    // wrap
    direction = fmodf(direction, CHASSIS_2PI);
    if (direction < 0) {
        direction += CHASSIS_2PI;
    }

    V = fmaxf(0.f, fminf(MAX_CHASSIS_V, V));
    if (V < CHASSIS_V_DEADZONE) {
        V = 0.f;
    }

    omega = fmaxf(-MAX_CHASSIS_OMEGA, fminf(MAX_CHASSIS_OMEGA, omega));
    if (fabsf(omega) < CHASSIS_OMEGA_DEADZONE) {
        omega = 0.f;
    }

    // set
    chassis->move_CS = move_CS;
    if (move_CS == CHASSIS_MOVE_MODE_LCS) {
        chassis->target_LCS_direction = direction;
    } else {
        chassis->target_WCS_direction = direction;
    }
    chassis->target_V = V;
    chassis->target_omega = omega;
}

void chassis_reset_WCS_and_odometry(chassis_t *chassis, float WCS_direction, float WCS_X, float WCS_Y)
{
    // wrap
    WCS_direction = fmodf(WCS_direction, CHASSIS_2PI);
    if (WCS_direction < 0) {
        WCS_direction += CHASSIS_2PI;
    }

    chassis->current_WCS_direction = WCS_direction;
    chassis->current_WCS_X = WCS_X;
    chassis->current_WCS_Y = WCS_Y;
}

// convert linear velocity (m/s) to motor speed (rpm)
inline static float motor_mps_to_rpm(float mps)
{
    return ((mps * 60.f) / (WHEEL_RADIUS * CHASSIS_2PI));
}

// convert motor speed (rpm) to linear velocity (m/s)
inline static float motor_rpm_to_mps(float rpm)
{
    return ((rpm * WHEEL_RADIUS * CHASSIS_2PI) / 60.f);
}

static void chassis_FK_and_odometry(chassis_t *chassis, float dt)
{
    float wheel_rpm[MOTOR_COUNT] = {0.f, 0.f, 0.f, 0.f};
    float wheel_v[MOTOR_COUNT] = {0.f, 0.f, 0.f, 0.f};

    for (int i = 0; i < MOTOR_COUNT; i++) {
        wheel_rpm[i] = chassis->motor[i].get_rpm(chassis->motor[i].ctx);
        wheel_v[i] = motor_rpm_to_mps(wheel_rpm[i]);
        if (fabsf(wheel_v[i]) < 0.001f) {
            wheel_v[i] = 0.0f;
        }
    }

    // LCS
    float ux = INV_SQRT8 * (-wheel_v[MOTOR_LF] + wheel_v[MOTOR_RF] +
                            wheel_v[MOTOR_RB] - wheel_v[MOTOR_LB]);
    float uy = INV_SQRT8 * (wheel_v[MOTOR_LF] + wheel_v[MOTOR_RF] -
                            wheel_v[MOTOR_RB] - wheel_v[MOTOR_LB]);
    float omega = (wheel_v[MOTOR_LF] + wheel_v[MOTOR_RF] + wheel_v[MOTOR_RB] + wheel_v[MOTOR_LB]) /
                  (4.f * CHASSIS_R);
    chassis->current_LCS_direction = atan2f(uy, ux);

    // common
    chassis->current_omega = omega;
    chassis->current_V = hypotf(ux, uy);

    // WCS and odometry
    float theta_mid = chassis->current_WCS_direction + omega * dt * 0.5f; // mid-point method for better accuracy
    float cos_theta = cosf(theta_mid);
    float sin_theta = sinf(theta_mid);
    chassis->current_WCS_Vx = ux * cos_theta - uy * sin_theta;
    chassis->current_WCS_Vy = ux * sin_theta + uy * cos_theta;
    chassis->current_WCS_X += chassis->current_WCS_Vx * dt;
    chassis->current_WCS_Y += chassis->current_WCS_Vy * dt;

    chassis->current_WCS_direction += omega * dt;
    chassis->current_WCS_direction = fmodf(chassis->current_WCS_direction, CHASSIS_2PI);
    if (chassis->current_WCS_direction < 0) {
        chassis->current_WCS_direction += CHASSIS_2PI;
    }
}

static void chassis_IK(chassis_t *chassis, float wheel_rpm[])
{
    float wheel_v[MOTOR_COUNT] = {0.f, 0.f, 0.f, 0.f};
    float ux = 0.f;
    float uy = 0.f;
    float scale_omega = CHASSIS_R * chassis->target_omega;

    if (chassis->move_CS == CHASSIS_MOVE_MODE_LCS) {
        ux = chassis->target_V * cosf(chassis->target_LCS_direction);
        uy = chassis->target_V * sinf(chassis->target_LCS_direction);
    } else {
        float vx = chassis->target_V * cosf(chassis->target_WCS_direction);
        float vy = chassis->target_V * sinf(chassis->target_WCS_direction);
        float cos_theta = cosf(chassis->current_WCS_direction);
        float sin_theta = sinf(chassis->current_WCS_direction);

        ux = cos_theta * vx + sin_theta * vy;
        uy = -sin_theta * vx + cos_theta * vy;
    }

    float scale_ux = INV_SQRT2 * ux;
    float scale_uy = INV_SQRT2 * uy;

    wheel_v[MOTOR_LF] = -scale_ux + scale_uy + scale_omega;
    wheel_v[MOTOR_RF] = scale_ux + scale_uy + scale_omega;
    wheel_v[MOTOR_RB] = scale_ux - scale_uy + scale_omega;
    wheel_v[MOTOR_LB] = -scale_ux - scale_uy + scale_omega;

    // prevent saturation and distortion
    float max_wheel_v = motor_rpm_to_mps(MAX_WHEEL_RPM);
    float max_abs_wheel_v = 0.f;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        float abs_wheel_v = fabsf(wheel_v[i]);
        if (abs_wheel_v > max_abs_wheel_v) {
            max_abs_wheel_v = abs_wheel_v;
        }
    }

    if (max_abs_wheel_v > max_wheel_v) {
        float scale = max_wheel_v / max_abs_wheel_v;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            wheel_v[i] *= scale;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        wheel_rpm[i] = motor_mps_to_rpm(wheel_v[i]);
    }
}

void chassis_update(chassis_t *chassis, float dt)
{
    chassis_FK_and_odometry(chassis, dt);

    float wheel_rpm[MOTOR_COUNT] = {0.f, 0.f, 0.f, 0.f};
    chassis_IK(chassis, wheel_rpm);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        chassis->motor[i].set_rpm(chassis->motor[i].ctx, wheel_rpm[i]);
        chassis->motor[i].update(chassis->motor[i].ctx, dt);
    }
}
