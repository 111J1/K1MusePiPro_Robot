#ifndef K1MUSE_CONTROL_CORE__GRASP_PROFILE_H_
#define K1MUSE_CONTROL_CORE__GRASP_PROFILE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    K1_GRASP_MODE_SIDE = 0,
    K1_GRASP_MODE_TOP = 1,
    K1_GRASP_MODE_OBLIQUE = 2,
} k1_grasp_mode_t;

typedef enum {
    K1_GRIPPER_FIXED_ANGLE = 0,
    K1_GRIPPER_CONTACT_SWITCH = 1,
    K1_GRIPPER_VISION_VERIFY = 2,
} k1_gripper_strategy_t;

typedef struct {
    int32_t mode;
    int32_t gripper_strategy;
    float gripper_open_rad;
    float gripper_close_rad;
    float roll_rad;
    float pitch_rad;
    float preferred_arm_x;
    float preferred_arm_y;
    float preferred_arm_z;
    float approach_distance_m;
    float retreat_distance_m;
    float lift_distance_m;
    float transport_max_v;
    float transport_max_omega;
    uint32_t settle_time_ms;
    uint32_t calibrated;
} k1_grasp_variant_t;

typedef enum {
    K1_PROFILE_OK = 0,
    K1_PROFILE_NULL,
    K1_PROFILE_NOT_CALIBRATED,
    K1_PROFILE_INVALID_MODE,
    K1_PROFILE_INVALID_ANGLE,
    K1_PROFILE_INVALID_DISTANCE,
} k1_profile_status_t;

k1_profile_status_t k1_grasp_profile_validate(
    const k1_grasp_variant_t *profile,
    float gripper_min_rad, float gripper_max_rad);

#ifdef __cplusplus
}
#endif

#endif
