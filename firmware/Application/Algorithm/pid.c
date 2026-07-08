/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * @Files: pid.c/h
 *
 * @Author: Ye Jinyi
 *
 * @First  Edit Date: 2025.1.15
 * @Latest Edit Date: 2025.1.18
 *
 * @illustrate:
 *
 * @attention:
 *
 *-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Including Files
 * -------------------------------------------------------------------------------------------------------------------------------------------------
 */
#include "pid.h"
#include "math.h"

/* External Variables Declarations
 * ----------------------------------------------------------------------------------------------------------------------------------
 */

/* Define Exported Variables
 * ----------------------------------------------------------------------------------------------------------------------------------------
 */

/* Define Privated Variables
 * ----------------------------------------------------------------------------------------------------------------------------------------
 */

/* Define Privated Functions
 * ----------------------------------------------------------------------------------------------------------------------------------------
 */
#define pid_constrain(x, min, max) ((x > max) ? max : (x < min ? min : x))

/* Define Exported Functions
 * ----------------------------------------------------------------------------------------------------------------------------------------
 */

inline void position_pid_ctrl(position_pid_ctrl_t *pid)
{
    /* 计算误差 */
    pid->err = pid->target - pid->measure;

    /* 积分 */
    pid->integral += pid->err;
    pid->integral = pid_constrain(pid->integral, pid->integral_min, pid->integral_max);

    /* p i d 输出项计算 */
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
    pid->dout = pid->kd * (pid->err - pid->last_err);

    /* 累加pid输出值 */
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = pid_constrain(pid->out, pid->out_min, pid->out_max);

    /* 记录上次误差值 */
    pid->last_err = pid->err;
}

inline void position_pid_clear_out(position_pid_ctrl_t *pid)
{
    pid->measure = 0;
    pid->target = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
    pid->last_out = 0;
}

inline void incremental_pid_ctrl(incremental_pid_ctrl_t *pid)
{
    /* 计算误差 */
    pid->err = pid->target - pid->measure;

    /* 增量计算 */
    pid->delta_u = pid->kp * (pid->err - pid->last_err)                          // 比例增量
                   + pid->ki * pid->err                                          // 积分增量
                   + pid->kd * (pid->err - 2.f * pid->last_err + pid->prev_err); // 微分增量

    /* 累加输出 */
    pid->out = pid->last_out + pid->delta_u;

    /* 限幅输出 */
    pid->out = pid_constrain(pid->out, pid->out_min, pid->out_max);

    /* 保存状态，供下一次迭代使用 */
    pid->prev_err = pid->last_err; // 更新上上次误差
    pid->last_err = pid->err;      // 更新上次误差
    pid->last_out = pid->out;      // 更新上次输出
}

inline void incremental_pid_clear_out(incremental_pid_ctrl_t *pid)
{
    pid->measure = 0;
    pid->target = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->prev_err = 0;
    pid->delta_u = 0;
    pid->out = 0;
    pid->last_out = 0;
}
