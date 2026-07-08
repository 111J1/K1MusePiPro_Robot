#ifndef _PID_H
#define _PID_H

/* Including Files
 * --------------------------------------------------------------------------------------------------------------------------------------------------
 */

/* Macro
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

/* Define Exported Variables Type
 * -----------------------------------------------------------------------------------------------------------------------------------
 */
typedef struct {
    float target;           // 目标值
    float measure;          // 测量值
    float err;              // 当前误差
    float last_err;         // 上一次误差
    float kp, ki, kd;       // 比例、积分、微分系数
    float pout, iout, dout; // 比例、积分、微分输出
    float out;              // 当前输出
    float last_out;         // 上一次输出
    float integral;         // 积分
    float integral_min;     // 积分下限
    float integral_max;     // 积分上限
    float out_min;          // 输出下限
    float out_max;          // 输出上限
} position_pid_ctrl_t;

typedef struct {
    float target;     // 目标值
    float measure;    // 测量值
    float err;        // 当前误差
    float last_err;   // 上一次误差
    float prev_err;   // 上上次误差
    float kp, ki, kd; // 比例、积分、微分系数
    float delta_u;    // 增量输出
    float out;        // 当前输出
    float last_out;   // 上一次输出
    float out_min;    // 输出下限
    float out_max;    // 输出上限
} incremental_pid_ctrl_t;

/* Define Privated Variables Type
 * -----------------------------------------------------------------------------------------------------------------------------------
 */

/* Exported Variables Declarations
 * ----------------------------------------------------------------------------------------------------------------------------------
 */

/* Exported Functions Declarations
 * ----------------------------------------------------------------------------------------------------------------------------------
 */
void position_pid_ctrl(position_pid_ctrl_t *pid);
void position_pid_clear_out(position_pid_ctrl_t *pid);
void incremental_pid_ctrl(incremental_pid_ctrl_t *pid);
void incremental_pid_clear_out(incremental_pid_ctrl_t *pid);

#endif
