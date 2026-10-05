#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "motor.h"
#include "motor_ctrl.h"

static const char *TAG = "motor_ctrl";

#define CTRL_PERIOD_MS   20      // 控制周期
#define SWEEP_STEP_MS    2000    

/* 慢 → 中 → 快 → 中*/
static const int SWEEP_LADDER[] = {30, 60, 90, 60};
#define SWEEP_LEN (sizeof(SWEEP_LADDER) / sizeof(SWEEP_LADDER[0]))

static volatile motor_mode_t s_mode = MOTOR_MODE_STOP;
static volatile motor_mode_t s_prev_mode = MOTOR_MODE_SWEEP;  /* 供 ON 恢复 */
static volatile int s_manual_pct = 0;
static volatile int s_target = 0;      

static float target_to_duty(int target_pct)
{
    return (float)target_pct / 100.0f;
}

static void ctrl_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t level_start = last_wake;
    size_t level = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        int target = 0;
        switch (s_mode) {
        case MOTOR_MODE_SWEEP:
            if (xTaskGetTickCount() - level_start >= pdMS_TO_TICKS(SWEEP_STEP_MS)) {
                level = (level + 1) % SWEEP_LEN;
                level_start = xTaskGetTickCount();
                ESP_LOGI(TAG, "渐变档位 %u/%u → %d%%",
                         (unsigned)(level + 1), (unsigned)SWEEP_LEN, SWEEP_LADDER[level]);
            }
            target = SWEEP_LADDER[level];
            break;

        case MOTOR_MODE_MANUAL:
            target = s_manual_pct;
            break;

        case MOTOR_MODE_STOP:
        default:
            target = 0;
            break;
        }

        s_target = target;
        motor_set_duty(target_to_duty(target));
    }
}

esp_err_t motor_ctrl_start(void)
{
    s_mode = MOTOR_MODE_SWEEP;
    BaseType_t ok = xTaskCreate(ctrl_task, "motor_ctrl", 3072, NULL, 10, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "控制任务创建失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "控制任务已启动，周期 %dms，初始为自动渐变", CTRL_PERIOD_MS);
    return ESP_OK;
}

void motor_ctrl_set_manual(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_manual_pct = percent;
    s_mode = (percent == 0) ? MOTOR_MODE_STOP : MOTOR_MODE_MANUAL;
    ESP_LOGI(TAG, "切到手动模式，目标 %d%%", percent);
}

void motor_ctrl_set_sweep(void)
{
    s_mode = MOTOR_MODE_SWEEP;
    ESP_LOGI(TAG, "切回自动渐变模式");
}

void motor_ctrl_stop(void)
{
    if (s_mode != MOTOR_MODE_STOP) {
        s_prev_mode = s_mode;
    }
    s_mode = MOTOR_MODE_STOP;
    motor_stop();
    ESP_LOGI(TAG, "电机停止");
}

void motor_ctrl_resume(void)
{
    s_mode = s_prev_mode;
    ESP_LOGI(TAG, "恢复运行，模式：%s",
             s_mode == MOTOR_MODE_MANUAL ? "手动" : "自动渐变");
}

void motor_ctrl_set_dir(bool forward)
{
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
