#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float i_sum;
    float i_limit;              /* 积分限幅，防止积分饱和 */
    float out_min, out_max;
    float err_last;
    float target;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd);
void pid_set_target(pid_ctrl_t *p, float target);
void pid_set_output_limit(pid_ctrl_t *p, float out_min, float out_max);
void pid_set_integral_limit(pid_ctrl_t *p, float limit);
void pid_reset(pid_ctrl_t *p);

/* 传入实测值，返回限幅后的输出 */
float pid_update(pid_ctrl_t *p, float measured);

#ifdef __cplusplus
}
#endif
