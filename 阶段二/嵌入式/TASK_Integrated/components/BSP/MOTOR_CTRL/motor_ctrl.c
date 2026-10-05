#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "motor.h"
#include "motor_ctrl.h"
#include "encoder.h"
#include "pid.h"

static const char *TAG = "motor_ctrl";

#define CTRL_PERIOD_MS   20
#define SWEEP_STEP_MS    2000
#define CLOSED_LOG_MS    200

static const int SWEEP_LADDER[] = {30, 60, 90, 60};
#define SWEEP_LEN (sizeof(SWEEP_LADDER) / sizeof(SWEEP_LADDER[0]))

/* 占位参数，要按实物整定 */
#define PID_KP_DEFAULT   0.0025f
#define PID_KI_DEFAULT   0.0015f
#define PID_KD_DEFAULT   0.0f

static bool s_inited;               /* 控制任务没起来的话，改模式不会有任何效果 */
static volatile motor_mode_t s_mode = MOTOR_MODE_STOP;
static volatile motor_mode_t s_prev_mode = MOTOR_MODE_SWEEP;
static volatile int s_manual_pct;
static volatile int s_target;
static volatile int s_closed_rpm;   /* 单独存，STOP 会把 s_target 清零 */
static pid_ctrl_t s_pid;

static bool ready(const char *what)
{
    if (s_inited) {
        return true;
    }
    ESP_LOGE(TAG, "%s: motor_ctrl_start() 没调用，忽略", what);
    return false;
}

static float target_to_duty(int target_pct)
{
    return (float)target_pct / 100.0f;
}

static void ctrl_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t level_start = last_wake;
    TickType_t log_start = last_wake;
    size_t level = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        switch (s_mode) {
        case MOTOR_MODE_SWEEP:
            if (xTaskGetTickCount() - level_start >= pdMS_TO_TICKS(SWEEP_STEP_MS)) {
                level = (level + 1) % SWEEP_LEN;
                level_start = xTaskGetTickCount();
            }
            s_target = SWEEP_LADDER[level];
            motor_set_duty(target_to_duty(s_target));
            break;

        case MOTOR_MODE_MANUAL:
            s_target = s_manual_pct;
            motor_set_duty(target_to_duty(s_target));
            break;

        case MOTOR_MODE_CLOSED:
            motor_set_duty(pid_update(&s_pid, encoder_get_rpm()));
            break;

        case MOTOR_MODE_STOP:
        default:
            s_target = 0;
            motor_set_duty(0.0f);
            break;
        }

        if (s_mode == MOTOR_MODE_CLOSED &&
            xTaskGetTickCount() - log_start >= pdMS_TO_TICKS(CLOSED_LOG_MS)) {
            log_start = xTaskGetTickCount();
            ESP_LOGI(TAG, "目标 %.0f → 实际 %.1f", s_pid.target, encoder_get_rpm());
        }
    }
}

esp_err_t motor_ctrl_start(void)
{
    pid_init(&s_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);

    s_mode = MOTOR_MODE_STOP;       /* 上电不动，发 MOTOR ON 才转 */

    if (xTaskCreate(ctrl_task, "motor_ctrl", 3072, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "控制任务创建失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "控制任务已启动，周期 %dms", CTRL_PERIOD_MS);
    s_inited = true;
    return ESP_OK;
}

void motor_ctrl_set_manual(int percent)
{
    if (!ready("set_manual")) return;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_manual_pct = percent;
    s_mode = (percent == 0) ? MOTOR_MODE_STOP : MOTOR_MODE_MANUAL;
    ESP_LOGI(TAG, "开环 %d%%", percent);
}

void motor_ctrl_set_sweep(void)
{
    if (!ready("set_sweep")) return;
    s_mode = MOTOR_MODE_SWEEP;
    ESP_LOGI(TAG, "自动渐变");
}

void motor_ctrl_set_closed(int target_rpm)
{
    if (!ready("set_closed")) return;
    if (target_rpm <= 0) {
        motor_ctrl_set_manual(0);
        return;
    }
    s_closed_rpm = target_rpm;
    s_target = target_rpm;
    pid_set_target(&s_pid, (float)target_rpm);
    pid_reset(&s_pid);
    s_mode = MOTOR_MODE_CLOSED;
    ESP_LOGI(TAG, "闭环目标 %d RPM", target_rpm);
}

void motor_ctrl_stop(void)
{
    if (!ready("stop")) return;
    if (s_mode != MOTOR_MODE_STOP) {
        s_prev_mode = s_mode;
    }
    s_mode = MOTOR_MODE_STOP;
    motor_stop();
    ESP_LOGI(TAG, "电机停止");
}

void motor_ctrl_resume(void)
{
    if (!ready("resume")) return;
    if (s_prev_mode == MOTOR_MODE_CLOSED && s_closed_rpm > 0) {
        motor_ctrl_set_closed(s_closed_rpm);
        return;
    }
    s_mode = s_prev_mode;
    ESP_LOGI(TAG, "恢复运行");
}

void motor_ctrl_set_dir(bool forward)
{
    if (!ready("set_dir")) return;
    motor_set_dir(forward);
}

motor_mode_t motor_ctrl_get_mode(void)
{
    return s_mode;
}

int motor_ctrl_get_target(void)
{
    return s_target;
}

float motor_ctrl_get_rpm(void)
{
    return encoder_get_rpm();
}
