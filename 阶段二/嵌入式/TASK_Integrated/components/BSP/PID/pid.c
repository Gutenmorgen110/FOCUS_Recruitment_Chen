/* 位置式 PID，D 项用 e(k)-e(k-1) 的常规符号约定 */
#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_sum = 0.0f;
    p->i_limit = 0.0f;
    p->out_min = -1.0f;
    p->out_max = 1.0f;
    p->err_last = 0.0f;
    p->target = 0.0f;
}

void pid_set_target(pid_ctrl_t *p, float target)
{
    p->target = target;
}

void pid_set_output_limit(pid_ctrl_t *p, float out_min, float out_max)
{
    p->out_min = out_min;
    p->out_max = out_max;
}

void pid_set_integral_limit(pid_ctrl_t *p, float limit)
{
    p->i_limit = limit;
}

void pid_reset(pid_ctrl_t *p)
{
    p->i_sum = 0.0f;
    p->err_last = 0.0f;
}

float pid_update(pid_ctrl_t *p, float measured)
{
    float err = p->target - measured;
    float d = err - p->err_last;
    p->err_last = err;

    float out = p->kp * err + p->ki * p->i_sum + p->kd * d;

    /* 输出没饱和才继续积分，避免饱和后积分越攒越大 */
    if ((out <= p->out_max && out >= p->out_min) || (err * out < 0.0f)) {
        p->i_sum += err;
        if (p->i_limit > 0.0f) {
            if (p->i_sum > p->i_limit) {
                p->i_sum = p->i_limit;
            } else if (p->i_sum < -p->i_limit) {
                p->i_sum = -p->i_limit;
            }
        }
    }

    out = p->kp * err + p->ki * p->i_sum + p->kd * d;

    if (out > p->out_max) {
        out = p->out_max;
    } else if (out < p->out_min) {
        out = p->out_min;
    }
    return out;
}
