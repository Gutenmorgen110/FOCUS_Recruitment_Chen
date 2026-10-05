#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float i_sum;
    float err_last;
    float target;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd);
void pid_set_target(pid_ctrl_t *p, float target);
void pid_reset(pid_ctrl_t *p);

/* 传入实测值，返回 0~1 的占空比 */
float pid_update(pid_ctrl_t *p, float measured);

#ifdef __cplusplus
}
#endif
