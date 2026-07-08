#include "so101_algorithm.h"
#include "mdl_so101_arm_config.h"
#include <math.h>
#include <string.h>

#define ROBOT_EPSILON (1.0e-12f)
#define SO101_CHAIN_JOINT_COUNT (6U)
#define SO101_SOLVE_EPSILON (1.0e-9f)
#define SO101_PI (3.14159265358979323846f)
#define SO101_TWO_PI (6.28318530717958647692f)
#define SO101_POSE_PITCH_TOLERANCE_RAD (0.03f)
#define SO101_POSE_D_REACH_EPS (1.0e-5f)
#define SO101_POSE_BRANCH_ELBOW_POS (1U)
#define SO101_POSE_BRANCH_ELBOW_NEG (2U)

#define SO101_GEOM_L1_M (0.116000021f)
#define SO101_GEOM_L2_M (0.135000185f)
#define SO101_GEOM_ALPHA1_RAD (1.327013107f)
#define SO101_GEOM_DELTA_RAD (-1.288485110f)
#define SO101_GEOM_TOOL_M (0.0981274f)
#define SO101_GEOM_WRIST_M (0.0611f)
#define SO101_GEOM_WRIST_SIDE_M (0.0181f)
#define SO101_GEOM_J1_ORIGIN_X_M (0.0388353f)
#define SO101_GEOM_J1_ORIGIN_Y_M (-8.97657e-09f)
#define SO101_GEOM_SHOULDER_SIDE_M (-0.018277863f)

static void robot_vec3_set(vec3f_t *v, float x, float y, float z)
{
    if (v != 0) 
    {
        v->x = x;
        v->y = y;
        v->z = z;
    }
}

static void robot_vec3_cross(const vec3f_t *a, const vec3f_t *b, vec3f_t *out)
{
    vec3f_t r;

    r.x = a->y * b->z - a->z * b->y;
    r.y = a->z * b->x - a->x * b->z;
    r.z = a->x * b->y - a->y * b->x;
    *out = r;
}

static float robot_vec3_norm(const vec3f_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static void robot_vec3_normalize(vec3f_t *v)
{
    float n = robot_vec3_norm(v);

    if (n < ROBOT_EPSILON) 
    {
        robot_vec3_set(v, 0.0f, 0.0f, 1.0f);
    } 
    else
    {
        v->x /= n;
        v->y /= n;
        v->z /= n;
    }
}

static void robot_mat4_identity(mat4f_t *m)
{
    memset(m, 0, sizeof(*m));
    m->m[0][0] = 1.0f;
    m->m[1][1] = 1.0f;
    m->m[2][2] = 1.0f;
    m->m[3][3] = 1.0f;
}

static void robot_mat4_mul(const mat4f_t *a, const mat4f_t *b, mat4f_t *out)
{
    mat4f_t r;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++) 
        {
            r.m[i][j] = 0.0f;
            for (uint8_t k = 0U; k < 4U; k++) 
            {
                r.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    *out = r;
}

static void rpy_to_rotation(const float rpy[3], float r[3][3])
{
    float roll = rpy[0];
    float pitch = rpy[1];
    float yaw = rpy[2];
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    r[0][0] = cy * cp;
    r[0][1] = cy * sp * sr - sy * cr;
    r[0][2] = cy * sp * cr + sy * sr;
    r[1][0] = sy * cp;
    r[1][1] = sy * sp * sr + cy * cr;
    r[1][2] = sy * sp * cr - cy * sr;
    r[2][0] = -sp;
    r[2][1] = cp * sr;
    r[2][2] = cp * cr;
}

static void robot_mat4_make_transform(const float xyz[3], const float rpy[3], mat4f_t *out)
{
    float r[3][3];

    robot_mat4_identity(out);
    rpy_to_rotation(rpy, r);
    for (uint8_t i = 0U; i < 3U; i++) 
    {
        for (uint8_t j = 0U; j < 3U; j++) 
        {
            out->m[i][j] = r[i][j];
        }
    }
    out->m[0][3] = xyz[0];
    out->m[1][3] = xyz[1];
    out->m[2][3] = xyz[2];
}

static void robot_mat4_axis_angle(const float axis[3], float angle, mat4f_t *out)
{
    vec3f_t a = {axis[0], axis[1], axis[2]};
    float c = cosf(angle);
    float s = sinf(angle);
    float v = 1.0f - c;
    float x;
    float y;
    float z;

    robot_vec3_normalize(&a);
    x = a.x;
    y = a.y;
    z = a.z;

    robot_mat4_identity(out);
    out->m[0][0] = x * x * v + c;
    out->m[0][1] = x * y * v - z * s;
    out->m[0][2] = x * z * v + y * s;
    out->m[1][0] = x * y * v + z * s;
    out->m[1][1] = y * y * v + c;
    out->m[1][2] = y * z * v - x * s;
    out->m[2][0] = x * z * v - y * s;
    out->m[2][1] = y * z * v + x * s;
    out->m[2][2] = z * z * v + c;
}

static void robot_mat4_transform_axis(const mat4f_t *m, const float axis[3], vec3f_t *out)
{
    out->x = m->m[0][0] * axis[0] + m->m[0][1] * axis[1] + m->m[0][2] * axis[2];
    out->y = m->m[1][0] * axis[0] + m->m[1][1] * axis[1] + m->m[1][2] * axis[2];
    out->z = m->m[2][0] * axis[0] + m->m[2][1] * axis[1] + m->m[2][2] * axis[2];
    robot_vec3_normalize(out);
}

static void robot_mat4_get_translation(const mat4f_t *m, vec3f_t *out)
{
    robot_vec3_set(out, m->m[0][3], m->m[1][3], m->m[2][3]);
}

static float robot_clampf(float x, float lower, float upper)
{
    if (x > upper) 
    {
        return upper;
    }
    if (x < lower) 
    {
        return lower;
    }
    return x;
}

static float robot_wrap_pi(float angle)
{
    while (angle > SO101_PI)
    {
        angle -= SO101_TWO_PI;
    }
    while (angle < -SO101_PI)
    {
        angle += SO101_TWO_PI;
    }
    return angle;
}

static float robot_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float robot_vec3_dot(const vec3f_t *a, const vec3f_t *b)
{
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

static void robot_vec3_sub(const vec3f_t *a, const vec3f_t *b, vec3f_t *out)
{
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

static void robot_vec3_scale_add(vec3f_t *out, const vec3f_t *v, float scale)
{
    out->x += v->x * scale;
    out->y += v->y * scale;
    out->z += v->z * scale;
}

static const robot_joint_param_t so101_joints[SO101_CHAIN_JOINT_COUNT] = 
{
    {{0.0388353f, -8.97657e-09f, 0.0624f},
     {3.14159f, 4.18253e-17f, -3.14159f},
     {0.0f, 0.0f, 1.0f},
     SO101_ARM_JOINT_1_MIN_RAD,
     SO101_ARM_JOINT_1_MAX_RAD,
     1U},
    {{-0.0303992f, -0.0182778f, -0.0542f},
     {-1.5708f, -1.5708f, 0.0f},
     {0.0f, 0.0f, 1.0f},
     SO101_ARM_JOINT_2_MIN_RAD,
     SO101_ARM_JOINT_2_MAX_RAD,
     1U},
    {{-0.11257f, -0.028f, 1.73763e-16f},
     {-3.63608e-16f, 8.74301e-16f, 1.5708f},
     {0.0f, 0.0f, 1.0f},
     SO101_ARM_JOINT_3_MIN_RAD,
     SO101_ARM_JOINT_3_MAX_RAD,
     1U},
    {{-0.1349f, 0.0052f, 3.62355e-17f},
     {4.02456e-15f, 8.67362e-16f, -1.5708f},
     {0.0f, 0.0f, 1.0f},
     SO101_ARM_JOINT_4_MIN_RAD,
     SO101_ARM_JOINT_4_MAX_RAD,
     1U},
    {{5.55112e-17f, -0.0611f, 0.0181f},
     {1.5708f, 0.0486795f, 3.14159f},
     {0.0f, 0.0f, 1.0f},
     SO101_ARM_JOINT_5_MIN_RAD,
     SO101_ARM_JOINT_5_MAX_RAD,
     1U},
    {{0.0f, 0.0f, -0.0981274f},
     {0.0f, 3.14159f, 0.0f},
     {0.0f, 0.0f, 0.0f},
     0.0f,
     0.0f,
     0U},
};

static const robot_model_t so101_model = 
{
    so101_joints,
    SO101_CHAIN_JOINT_COUNT,
    SO101_ACTIVE_JOINT_COUNT,
};

const robot_model_t *so101_model_get(void)
{
    return &so101_model;
}

static float robot_joint_limit_margin(const float q[SO101_ACTIVE_JOINT_COUNT])
{
    float min_margin = 1000.0f;

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++)
    {
        float lower_margin = q[i] - so101_joints[i].lower;
        float upper_margin = so101_joints[i].upper - q[i];
        float margin = (lower_margin < upper_margin) ? lower_margin : upper_margin;

        if (margin < min_margin)
        {
            min_margin = margin;
        }
    }

    return min_margin;
}

void so101_ik_default_options(robot_ik_options_t *opts)
{
    if (opts != 0) 
    {
        opts->max_iter = 60U;
        opts->tolerance_m = 0.003f;
        opts->damping = 0.05f;
        opts->max_step_rad = 0.04f;
    }
}

static robot_status_t model_fk_compute(const robot_model_t *model,
                                       const float *q,
                                       mat4f_t *tip_t,
                                       robot_joint_frame_t *frames)
{
    mat4f_t t;
    uint8_t active_index = 0U;

    if ((model == 0) || (q == 0) || (tip_t == 0)) 
    {
        return ROBOT_ERR_NULL;
    }

    robot_mat4_identity(&t);
    for (uint8_t i = 0U; i < model->joint_count; i++) 
    {
        const robot_joint_param_t *joint = &model->joints[i];
        mat4f_t origin_t;
        mat4f_t next_t;

        robot_mat4_make_transform(joint->xyz, joint->rpy, &origin_t);
        robot_mat4_mul(&t, &origin_t, &next_t);
        t = next_t;

        if (joint->is_active != 0U)
        {
            mat4f_t rot_t;

            if (active_index >= model->active_count) 
            {
                return ROBOT_ERR_RANGE;
            }

            if (frames != 0) 
            {
                robot_mat4_get_translation(&t, &frames[active_index].p);
                robot_mat4_transform_axis(&t, joint->axis, &frames[active_index].axis);
            }

            robot_mat4_axis_angle(joint->axis, q[active_index], &rot_t);
            robot_mat4_mul(&t, &rot_t, &next_t);
            t = next_t;
            active_index++;
        }
    }

    *tip_t = t;
    return ROBOT_OK;
}

robot_status_t so101_fk_compute(const float q[SO101_ACTIVE_JOINT_COUNT], mat4f_t *tip_t)
{
    return model_fk_compute(&so101_model, q, tip_t, 0);
}

robot_status_t so101_fk_compute_frames(const float q[SO101_ACTIVE_JOINT_COUNT],
                                       mat4f_t *tip_t,
                                       robot_joint_frame_t frames[SO101_ACTIVE_JOINT_COUNT])
{
    if (frames == 0) 
    {
        return ROBOT_ERR_NULL;
    }
    return model_fk_compute(&so101_model, q, tip_t, frames);
}

robot_status_t so101_position_jacobian(const float q[SO101_ACTIVE_JOINT_COUNT],
                                       float jacobian[3][SO101_ACTIVE_JOINT_COUNT])
{
    mat4f_t tip_t;
    vec3f_t tip;
    robot_joint_frame_t frames[SO101_ACTIVE_JOINT_COUNT];
    robot_status_t status;

    if ((q == 0) || (jacobian == 0))
    {
        return ROBOT_ERR_NULL;
    }

    status = so101_fk_compute_frames(q, &tip_t, frames);
    if (status != ROBOT_OK) 
    {
        return status;
    }

    robot_mat4_get_translation(&tip_t, &tip);
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) 
    {
        vec3f_t delta = 
        {
            tip.x - frames[i].p.x,
            tip.y - frames[i].p.y,
            tip.z - frames[i].p.z,
        };
        vec3f_t col;

        robot_vec3_cross(&frames[i].axis, &delta, &col);
        jacobian[0][i] = col.x;
        jacobian[1][i] = col.y;
        jacobian[2][i] = col.z;
    }

    return ROBOT_OK;
}

static robot_status_t solve_3x3(float a[3][3], const vec3f_t *b, vec3f_t *x)
{
    float m[3][4] = 
    {
        {a[0][0], a[0][1], a[0][2], b->x},
        {a[1][0], a[1][1], a[1][2], b->y},
        {a[2][0], a[2][1], a[2][2], b->z},
    };

    for (uint8_t col = 0U; col < 3U; col++) 
    {
        uint8_t pivot = col;
        float max_abs = fabsf(m[col][col]);

        for (uint8_t row = (uint8_t)(col + 1U); row < 3U; row++)
        {
            float v = fabsf(m[row][col]);
            if (v > max_abs) 
            {
                max_abs = v;
                pivot = row;
            }
        }

        if (max_abs < SO101_SOLVE_EPSILON) 
        {
            return ROBOT_ERR_SINGULAR;
        }

        if (pivot != col) {
            for (uint8_t k = col; k < 4U; k++) 
            {
                float tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }

        for (uint8_t row = (uint8_t)(col + 1U); row < 3U; row++) 
        {
            float factor = m[row][col] / m[col][col];
            for (uint8_t k = col; k < 4U; k++) 
            {
                m[row][k] -= factor * m[col][k];
            }
        }
    }

    x->z = m[2][3] / m[2][2];
    x->y = (m[1][3] - m[1][2] * x->z) / m[1][1];
    x->x = (m[0][3] - m[0][1] * x->y - m[0][2] * x->z) / m[0][0];
    return ROBOT_OK;
}

static void fill_info(robot_ik_info_t *info,
                      robot_status_t status,
                      uint16_t iterations,
                      float position_error_m)
{
    if (info != 0) 
    {
        info->status = status;
        info->iterations = iterations;
        info->position_error_m = position_error_m;
        info->pitch_error_rad = 0.0f;
        info->joint_limit_margin_rad = 0.0f;
        info->yaw_rad = 0.0f;
        info->side_error_m = 0.0f;
        info->solution_branch = 0U;
        info->used_refine = 0U;
    }
}

robot_status_t so101_position_ik(const vec3f_t *target_xyz,
                                 const float seed_q[SO101_ACTIVE_JOINT_COUNT],
                                 const robot_ik_options_t *opts,
                                 float out_q[SO101_ACTIVE_JOINT_COUNT],
                                 robot_ik_info_t *info)
{
    robot_ik_options_t local_opts;
    float q[SO101_ACTIVE_JOINT_COUNT];

    if ((target_xyz == 0) || (seed_q == 0) || (out_q == 0)) 
    {
        return ROBOT_ERR_NULL;
    }

    if (opts == 0) 
    {
        so101_ik_default_options(&local_opts);
    } 
    else 
    {
        local_opts = *opts;
    }

    memcpy(q, seed_q, sizeof(q));
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) 
    {
        q[i] = robot_clampf(q[i], so101_joints[i].lower, so101_joints[i].upper);
    }

    for (uint16_t iter = 0U; iter < local_opts.max_iter; iter++) 
    {
        mat4f_t tip_t;
        vec3f_t tip;
        vec3f_t err;
        float err_norm;
        float j[3][SO101_ACTIVE_JOINT_COUNT];
        float a[3][3];
        vec3f_t y;
        float dq[SO101_ACTIVE_JOINT_COUNT];
        float step_norm_sq = 0.0f;
        robot_status_t status;

        status = so101_fk_compute(q, &tip_t);
        if (status != ROBOT_OK) 
        {
            fill_info(info, status, iter, 0.0f);
            return status;
        }
        robot_mat4_get_translation(&tip_t, &tip);

        err.x = target_xyz->x - tip.x;
        err.y = target_xyz->y - tip.y;
        err.z = target_xyz->z - tip.z;
        err_norm = robot_vec3_norm(&err);
        if (err_norm < local_opts.tolerance_m) 
        {
            memcpy(out_q, q, sizeof(q));
            fill_info(info, ROBOT_OK, iter, err_norm);
            return ROBOT_OK;
        }

        status = so101_position_jacobian(q, j);
        if (status != ROBOT_OK) 
        {
            fill_info(info, status, iter, err_norm);
            return status;
        }

        for (uint8_t r = 0U; r < 3U; r++) 
        {
            for (uint8_t c = 0U; c < 3U; c++) 
            {
                a[r][c] = 0.0f;
                for (uint8_t k = 0U; k < SO101_ACTIVE_JOINT_COUNT; k++) 
                {
                    a[r][c] += j[r][k] * j[c][k];
                }
            }
        }
        a[0][0] += local_opts.damping * local_opts.damping;
        a[1][1] += local_opts.damping * local_opts.damping;
        a[2][2] += local_opts.damping * local_opts.damping;

        status = solve_3x3(a, &err, &y);
        if (status != ROBOT_OK) 
        {
            fill_info(info, status, iter, err_norm);
            return status;
        }

        for (uint8_t k = 0U; k < SO101_ACTIVE_JOINT_COUNT; k++) 
        {
            dq[k] = j[0][k] * y.x + j[1][k] * y.y + j[2][k] * y.z;
            step_norm_sq += dq[k] * dq[k];
        }

        if ((local_opts.max_step_rad > 0.0f) &&
            (step_norm_sq > local_opts.max_step_rad * local_opts.max_step_rad)) 
        {
            float scale = local_opts.max_step_rad / sqrtf(step_norm_sq);
            for (uint8_t k = 0U; k < SO101_ACTIVE_JOINT_COUNT; k++) 
            {
                dq[k] *= scale;
            }
        }

        for (uint8_t k = 0U; k < SO101_ACTIVE_JOINT_COUNT; k++) 
        {
            q[k] = robot_clampf(q[k] + dq[k], so101_joints[k].lower, so101_joints[k].upper);
        }
    }

    {
        mat4f_t tip_t;
        vec3f_t tip;
        vec3f_t err;
        float err_norm;

        (void)so101_fk_compute(q, &tip_t);
        robot_mat4_get_translation(&tip_t, &tip);
        err.x = target_xyz->x - tip.x;
        err.y = target_xyz->y - tip.y;
        err.z = target_xyz->z - tip.z;
        err_norm = robot_vec3_norm(&err);
        memcpy(out_q, q, sizeof(q));
        fill_info(info, ROBOT_ERR_NO_CONVERGE, local_opts.max_iter, err_norm);
    }

    return ROBOT_ERR_NO_CONVERGE;
}

typedef struct
{
    float q[SO101_ACTIVE_JOINT_COUNT];
    robot_ik_info_t info;
    float score;
    uint8_t valid;
} so101_pose_candidate_t;

static void so101_pose_fill_info(robot_ik_info_t *info,
                                 robot_status_t status,
                                 uint8_t branch,
                                 float position_error_m,
                                 float pitch_error_rad,
                                 float joint_limit_margin_rad,
                                 float yaw_rad,
                                 float side_error_m)
{
    if (info == 0)
    {
        return;
    }

    info->status = status;
    info->iterations = 1U;
    info->position_error_m = position_error_m;
    info->pitch_error_rad = pitch_error_rad;
    info->joint_limit_margin_rad = joint_limit_margin_rad;
    info->yaw_rad = yaw_rad;
    info->side_error_m = side_error_m;
    info->solution_branch = branch;
    info->used_refine = 0U;
}

static robot_status_t so101_pose_validate_candidate(const robot_pose_target_t *target_pose,
                                                    float yaw_rad,
                                                    float side_error_m,
                                                    so101_pose_candidate_t *candidate)
{
    mat4f_t tip_t;
    robot_joint_frame_t frames[SO101_ACTIVE_JOINT_COUNT];
    vec3f_t tip;
    vec3f_t roll_origin;
    vec3f_t tool_axis;
    vec3f_t err;
    float horizontal;
    float pitch_fk;
    robot_status_t status;

    status = so101_fk_compute_frames(candidate->q, &tip_t, frames);
    if (status != ROBOT_OK)
    {
        so101_pose_fill_info(&candidate->info, status, candidate->info.solution_branch,
                             1000.0f, 1000.0f, 0.0f, yaw_rad, side_error_m);
        return status;
    }

    robot_mat4_get_translation(&tip_t, &tip);
    roll_origin = frames[SO101_JOINT_WRIST_ROLL].p;
    robot_vec3_sub(&tip, &roll_origin, &tool_axis);
    robot_vec3_normalize(&tool_axis);

    err.x = target_pose->position.x - tip.x;
    err.y = target_pose->position.y - tip.y;
    err.z = target_pose->position.z - tip.z;
    horizontal = sqrtf((tool_axis.x * tool_axis.x) + (tool_axis.y * tool_axis.y));
    pitch_fk = atan2f(-tool_axis.z, horizontal);

    so101_pose_fill_info(&candidate->info,
                         ROBOT_OK,
                         candidate->info.solution_branch,
                         robot_vec3_norm(&err),
                         robot_wrap_pi(pitch_fk - target_pose->pitch_rad),
                         robot_joint_limit_margin(candidate->q),
                         yaw_rad,
                         side_error_m);
    return ROBOT_OK;
}

static uint8_t so101_pose_joint_limits_ok(const float q[SO101_ACTIVE_JOINT_COUNT])
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++)
    {
        if ((q[i] < so101_joints[i].lower) || (q[i] > so101_joints[i].upper))
        {
            return 0U;
        }
    }
    return 1U;
}

static float so101_pose_joint_distance(const float q[SO101_ACTIVE_JOINT_COUNT],
                                       const float seed_q[SO101_ACTIVE_JOINT_COUNT])
{
    float score = 0.0f;

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++)
    {
        float diff = robot_wrap_pi(q[i] - seed_q[i]);
        score += diff * diff;
    }

    return score;
}

static void so101_pose_build_candidate(const robot_pose_target_t *target_pose,
                                       const float seed_q[SO101_ACTIVE_JOINT_COUNT],
                                       const robot_ik_options_t *opts,
                                       float yaw_rad,
                                       float q1_rad,
                                       float side_error_m,
                                       float planar_r,
                                       float planar_h,
                                       float beta,
                                       uint8_t branch,
                                       so101_pose_candidate_t *candidate)
{
    float sin_beta = sinf(beta);
    float cos_beta = cosf(beta);
    float a;

    candidate->valid = 0U;
    candidate->info.solution_branch = branch;

    a = atan2f(planar_h, planar_r) -
        atan2f(SO101_GEOM_L2_M * sin_beta,
               SO101_GEOM_L1_M + (SO101_GEOM_L2_M * cos_beta));

    candidate->q[SO101_JOINT_SHOULDER_PAN] = q1_rad;
    candidate->q[SO101_JOINT_SHOULDER_LIFT] = SO101_GEOM_ALPHA1_RAD - a;
    candidate->q[SO101_JOINT_ELBOW_FLEX] = SO101_GEOM_DELTA_RAD - beta;
    candidate->q[SO101_JOINT_WRIST_FLEX] =
        target_pose->pitch_rad -
        candidate->q[SO101_JOINT_SHOULDER_LIFT] -
        candidate->q[SO101_JOINT_ELBOW_FLEX];
    candidate->q[SO101_JOINT_WRIST_ROLL] = target_pose->roll_rad;

    if (so101_pose_joint_limits_ok(candidate->q) == 0U)
    {
        so101_pose_fill_info(&candidate->info, ROBOT_ERR_RANGE, branch,
                             1000.0f, 1000.0f,
                             robot_joint_limit_margin(candidate->q),
                             yaw_rad, side_error_m);
        return;
    }

    (void)so101_pose_validate_candidate(target_pose, yaw_rad, side_error_m, candidate);
    candidate->score = so101_pose_joint_distance(candidate->q, seed_q);
    candidate->valid =
        ((candidate->info.position_error_m <= opts->tolerance_m) &&
         (robot_absf(candidate->info.pitch_error_rad) <= SO101_POSE_PITCH_TOLERANCE_RAD)) ? 1U : 0U;
}

static void so101_pose_select_best(const so101_pose_candidate_t *candidate,
                                   so101_pose_candidate_t *best,
                                   uint8_t *has_best)
{
    if (candidate->valid == 0U)
    {
        return;
    }

    if ((*has_best == 0U) ||
        (candidate->score < best->score) ||
        ((robot_absf(candidate->score - best->score) < 1.0e-6f) &&
         (candidate->info.joint_limit_margin_rad > best->info.joint_limit_margin_rad)))
    {
        *best = *candidate;
        *has_best = 1U;
    }
}

static uint8_t so101_pose_select_yaw_about_j1(const vec3f_t *target_position,
                                              float *yaw_rad)
{
    float dx = target_position->x - SO101_GEOM_J1_ORIGIN_X_M;
    float dy = target_position->y - SO101_GEOM_J1_ORIGIN_Y_M;
    float radius = sqrtf((dx * dx) + (dy * dy));
    float side_target;
    float arg;
    float base_angle;
    float asin_arg;
    float yaw_a;
    float yaw_b;
    float q1_a;
    float q1_b;
    uint8_t a_ok;
    uint8_t b_ok;

    if (radius < ROBOT_EPSILON)
    {
        return 0U;
    }

    side_target = SO101_GEOM_SHOULDER_SIDE_M + SO101_GEOM_WRIST_SIDE_M;
    arg = side_target / radius;
    if ((arg > 1.0f) || (arg < -1.0f))
    {
        return 0U;
    }

    base_angle = atan2f(dy, dx);
    asin_arg = asinf(arg);
    yaw_a = robot_wrap_pi(base_angle - asin_arg);
    yaw_b = robot_wrap_pi(base_angle - (SO101_PI - asin_arg));
    q1_a = robot_wrap_pi(-yaw_a);
    q1_b = robot_wrap_pi(-yaw_b);
    a_ok = ((q1_a >= so101_joints[SO101_JOINT_SHOULDER_PAN].lower) &&
            (q1_a <= so101_joints[SO101_JOINT_SHOULDER_PAN].upper)) ? 1U : 0U;
    b_ok = ((q1_b >= so101_joints[SO101_JOINT_SHOULDER_PAN].lower) &&
            (q1_b <= so101_joints[SO101_JOINT_SHOULDER_PAN].upper)) ? 1U : 0U;

    if ((a_ok == 0U) && (b_ok == 0U))
    {
        return 0U;
    }

    if ((a_ok != 0U) &&
        ((b_ok == 0U) ||
         (robot_absf(robot_wrap_pi(yaw_a - base_angle)) <=
          robot_absf(robot_wrap_pi(yaw_b - base_angle)))))
    {
        *yaw_rad = yaw_a;
    }
    else
    {
        *yaw_rad = yaw_b;
    }

    return 1U;
}

robot_status_t so101_pose_ik(const robot_pose_target_t *target_pose,
                             const float seed_q[SO101_ACTIVE_JOINT_COUNT],
                             const robot_ik_options_t *opts,
                             float out_q[SO101_ACTIVE_JOINT_COUNT],
                             robot_ik_info_t *info)
{
    robot_ik_options_t local_opts;
    float radius;
    float yaw_rad;
    float q1_rad;
    vec3f_t er;
    vec3f_t et;
    vec3f_t tool_axis;
    vec3f_t wrist_flex_target;
    vec3f_t shoulder;
    vec3f_t d;
    float q_yaw[SO101_ACTIVE_JOINT_COUNT] = {0.0f};
    mat4f_t tip_t;
    robot_joint_frame_t frames[SO101_ACTIVE_JOINT_COUNT];
    float r;
    float h;
    float side_error_m;
    float cosine_beta;
    so101_pose_candidate_t candidate_pos;
    so101_pose_candidate_t candidate_neg;
    so101_pose_candidate_t best;
    uint8_t has_best = 0U;
    robot_status_t status;

    if ((target_pose == 0) || (seed_q == 0) || (out_q == 0))
    {
        return ROBOT_ERR_NULL;
    }

    if (opts == 0)
    {
        so101_ik_default_options(&local_opts);
    }
    else
    {
        local_opts = *opts;
    }
    if (local_opts.tolerance_m < 0.005f)
    {
        local_opts.tolerance_m = 0.005f;
    }

    radius = sqrtf((target_pose->position.x * target_pose->position.x) +
                   (target_pose->position.y * target_pose->position.y));
    if (radius < ROBOT_EPSILON)
    {
        so101_pose_fill_info(info, ROBOT_ERR_SINGULAR, 0U,
                             1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
        return ROBOT_ERR_SINGULAR;
    }

    if (so101_pose_select_yaw_about_j1(&target_pose->position, &yaw_rad) == 0U)
    {
        so101_pose_fill_info(info, ROBOT_ERR_RANGE, 0U,
                             1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
        return ROBOT_ERR_RANGE;
    }
    q1_rad = robot_wrap_pi(-yaw_rad);
    robot_vec3_set(&er, cosf(yaw_rad), sinf(yaw_rad), 0.0f);
    robot_vec3_set(&et, -sinf(yaw_rad), cosf(yaw_rad), 0.0f);
    robot_vec3_set(&tool_axis,
                   cosf(target_pose->pitch_rad) * er.x,
                   cosf(target_pose->pitch_rad) * er.y,
                   -sinf(target_pose->pitch_rad));

    wrist_flex_target = target_pose->position;
    robot_vec3_scale_add(&wrist_flex_target, &tool_axis,
                         -(SO101_GEOM_TOOL_M + SO101_GEOM_WRIST_M));
    robot_vec3_scale_add(&wrist_flex_target, &et, -SO101_GEOM_WRIST_SIDE_M);

    q_yaw[SO101_JOINT_SHOULDER_PAN] = q1_rad;
    if ((q1_rad < so101_joints[SO101_JOINT_SHOULDER_PAN].lower) ||
        (q1_rad > so101_joints[SO101_JOINT_SHOULDER_PAN].upper))
    {
        so101_pose_fill_info(info, ROBOT_ERR_RANGE, 0U,
                             1000.0f, 1000.0f,
                             robot_joint_limit_margin(q_yaw),
                             yaw_rad, 0.0f);
        return ROBOT_ERR_RANGE;
    }

    status = so101_fk_compute_frames(q_yaw, &tip_t, frames);
    if (status != ROBOT_OK)
    {
        so101_pose_fill_info(info, status, 0U, 1000.0f, 1000.0f, 0.0f, yaw_rad, 0.0f);
        return status;
    }

    shoulder = frames[SO101_JOINT_SHOULDER_LIFT].p;
    robot_vec3_sub(&wrist_flex_target, &shoulder, &d);
    r = robot_vec3_dot(&d, &er);
    h = d.z;
    side_error_m = robot_vec3_dot(&d, &et);

    cosine_beta = ((r * r) + (h * h) -
                   (SO101_GEOM_L1_M * SO101_GEOM_L1_M) -
                   (SO101_GEOM_L2_M * SO101_GEOM_L2_M)) /
                  (2.0f * SO101_GEOM_L1_M * SO101_GEOM_L2_M);
    if ((cosine_beta > (1.0f + SO101_POSE_D_REACH_EPS)) ||
        (cosine_beta < (-1.0f - SO101_POSE_D_REACH_EPS)))
    {
        so101_pose_fill_info(info, ROBOT_ERR_UNREACHABLE, 0U,
                             1000.0f, 1000.0f,
                             0.0f, yaw_rad, side_error_m);
        return ROBOT_ERR_UNREACHABLE;
    }
    cosine_beta = robot_clampf(cosine_beta, -1.0f, 1.0f);

    so101_pose_build_candidate(target_pose, seed_q, &local_opts, yaw_rad, q1_rad, side_error_m,
                               r, h,
                               acosf(cosine_beta), SO101_POSE_BRANCH_ELBOW_POS,
                               &candidate_pos);
    so101_pose_build_candidate(target_pose, seed_q, &local_opts, yaw_rad, q1_rad, side_error_m,
                               r, h,
                               -acosf(cosine_beta), SO101_POSE_BRANCH_ELBOW_NEG,
                               &candidate_neg);

    so101_pose_select_best(&candidate_pos, &best, &has_best);
    so101_pose_select_best(&candidate_neg, &best, &has_best);

    if (has_best != 0U)
    {
        memcpy(out_q, best.q, sizeof(best.q));
        if (info != 0)
        {
            *info = best.info;
        }
        return ROBOT_OK;
    }

    best = candidate_pos;
    if (candidate_neg.info.position_error_m < candidate_pos.info.position_error_m)
    {
        best = candidate_neg;
    }
    if (info != 0)
    {
        *info = best.info;
        info->status = (best.info.status == ROBOT_OK) ? ROBOT_ERR_NO_CONVERGE : best.info.status;
    }
    return (best.info.status == ROBOT_OK) ? ROBOT_ERR_NO_CONVERGE : best.info.status;
}
