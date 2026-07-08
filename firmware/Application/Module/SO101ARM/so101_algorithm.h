#ifndef _SO101_ALGORITHM_H_
#define _SO101_ALGORITHM_H_

#include <stdint.h>

#define ROBOT_MAX_JOINTS (6U)

#define SO101_ACTIVE_JOINT_COUNT (5U)

typedef enum
{
    ROBOT_OK = 0,
    ROBOT_ERR_NULL = -1,
    ROBOT_ERR_RANGE = -2,
    ROBOT_ERR_SINGULAR = -3,
    ROBOT_ERR_NO_CONVERGE = -4,
    ROBOT_ERR_UNREACHABLE = -5,
} robot_status_t;

typedef struct
{
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct
{
    float m[4][4];
} mat4f_t;

typedef struct
{
    float xyz[3];
    float rpy[3];
    float axis[3];
    float lower;
    float upper;
    uint8_t is_active;
} robot_joint_param_t;

typedef struct
{
    vec3f_t p;
    vec3f_t axis;
} robot_joint_frame_t;

typedef struct
{
    const robot_joint_param_t *joints;
    uint8_t joint_count;
    uint8_t active_count;
} robot_model_t;

typedef struct
{
    uint16_t max_iter;
    float tolerance_m;
    float damping;
    float max_step_rad;
} robot_ik_options_t;

typedef struct
{
    robot_status_t status;
    uint16_t iterations;
    float position_error_m;
    float pitch_error_rad;
    float joint_limit_margin_rad;
    float yaw_rad;
    float side_error_m;
    uint8_t solution_branch;
    uint8_t used_refine;
} robot_ik_info_t;

typedef struct
{
    vec3f_t position;
    float roll_rad;
    float pitch_rad;
} robot_pose_target_t;

typedef enum 
{
    SO101_JOINT_SHOULDER_PAN = 0,
    SO101_JOINT_SHOULDER_LIFT,
    SO101_JOINT_ELBOW_FLEX,
    SO101_JOINT_WRIST_FLEX,
    SO101_JOINT_WRIST_ROLL,
    SO101_JOINT_COUNT,
} so101_joint_index_t;

const robot_model_t *so101_model_get(void);
void so101_ik_default_options(robot_ik_options_t *opts);

robot_status_t so101_fk_compute(const float q[SO101_ACTIVE_JOINT_COUNT], mat4f_t *tip_t);
robot_status_t so101_fk_compute_frames(const float q[SO101_ACTIVE_JOINT_COUNT],
                                       mat4f_t *tip_t,
                                       robot_joint_frame_t frames[SO101_ACTIVE_JOINT_COUNT]);
robot_status_t so101_position_jacobian(const float q[SO101_ACTIVE_JOINT_COUNT],
                                       float jacobian[3][SO101_ACTIVE_JOINT_COUNT]);
robot_status_t so101_position_ik(const vec3f_t *target_xyz,
                                 const float seed_q[SO101_ACTIVE_JOINT_COUNT],
                                 const robot_ik_options_t *opts,
                                 float out_q[SO101_ACTIVE_JOINT_COUNT],
                                 robot_ik_info_t *info);
robot_status_t so101_pose_ik(const robot_pose_target_t *target_pose,
                             const float seed_q[SO101_ACTIVE_JOINT_COUNT],
                             const robot_ik_options_t *opts,
                             float out_q[SO101_ACTIVE_JOINT_COUNT],
                             robot_ik_info_t *info);

#endif /* _SO101_ALGORITHM_H_ */
