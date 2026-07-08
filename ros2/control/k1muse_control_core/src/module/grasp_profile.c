#include "k1muse_control_core/grasp_profile.h"

#include <math.h>

k1_profile_status_t k1_grasp_profile_validate(
    const k1_grasp_variant_t *profile,
    float gripper_min_rad, float gripper_max_rad)
{
    if (profile == NULL) {
        return K1_PROFILE_NULL;
    }
    if (profile->calibrated == 0U) {
        return K1_PROFILE_NOT_CALIBRATED;
    }
    if ((profile->mode < K1_GRASP_MODE_SIDE) ||
        (profile->mode > K1_GRASP_MODE_OBLIQUE)) {
        return K1_PROFILE_INVALID_MODE;
    }
    if ((!isfinite(profile->gripper_open_rad)) ||
        (!isfinite(profile->gripper_close_rad)) ||
        (profile->gripper_open_rad < gripper_min_rad) ||
        (profile->gripper_open_rad > gripper_max_rad) ||
        (profile->gripper_close_rad < gripper_min_rad) ||
        (profile->gripper_close_rad > gripper_max_rad)) {
        return K1_PROFILE_INVALID_ANGLE;
    }
    if ((!isfinite(profile->roll_rad)) || (!isfinite(profile->pitch_rad)) ||
        (!isfinite(profile->preferred_arm_x)) ||
        (!isfinite(profile->preferred_arm_y)) ||
        (!isfinite(profile->preferred_arm_z)) ||
        (!isfinite(profile->approach_distance_m)) ||
        (!isfinite(profile->retreat_distance_m)) ||
        (!isfinite(profile->lift_distance_m)) ||
        (profile->approach_distance_m < 0.0f) ||
        (profile->retreat_distance_m < 0.0f) ||
        (profile->lift_distance_m < 0.0f) ||
        (profile->transport_max_v <= 0.0f) ||
        (profile->transport_max_omega < 0.0f)) {
        return K1_PROFILE_INVALID_DISTANCE;
    }
    return K1_PROFILE_OK;
}
