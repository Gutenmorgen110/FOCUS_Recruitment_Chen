/* 位置式 PID，D 项用 e(k)-e(k-1) 的常规符号约定 */
#include "pid.h"

#define PID_I_LIMIT     200.0f      /* 积分限幅 */
#define PID_OUT_MAX     1.0f        /* 输出即占空比 */

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_sum = 0.0f;
    p->err_last = 0.0f;
    p->target = 0.0f;
}

void pid_set_target(pid_ctrl_t *p, float target)
{
    p->target = target;
}

void pid_reset(pid_ctrl_t *p)
{
    p->i_sum = 0.0f;
    p->err_last = 0.0f;
}

float pid_update(pid_ctrl_t *p, float measured)
{
    float err = p->target - measured;

    p->i_sum += err;
    if (p->i_sum > PID_I_LIMIT) {
        p->i_sum = PID_I_LIMIT;
    } else if (p->i_sum < -PID_I_LIMIT) {
        p->i_sum = -PID_I_LIMIT;
    }

    float out = p->kp * err + p->ki * p->i_sum + p->kd * (err - p->err_last);
    p->err_last = err;

    if (out > PID_OUT_MAX) {
        out = PID_OUT_MAX;
    } else if (out < 0.0f) {
        out = 0.0f;
    }
    return out;
}
