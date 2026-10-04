/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "hal/uart_types.h"
#include <ctype.h>
#include "driver/ledc.h"
#include "esp_err.h"
static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: uart0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

//ledc
#define LEDC_LS_TIMER          LEDC_TIMER_0
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO       (8)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)
#define LEDC_MAX_DUTY   ((1 << 13) - 1)


static QueueHandle_t uart0_queue;
static bool is_led_on = false;
static uint8_t brightness = 0;

static void update_led(void) {
  uint32_t duty = is_led_on ? (uint32_t)brightness * LEDC_MAX_DUTY / 100 : 0;
  ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, duty);
  ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);

}
//实现字符串解码，并将参数更新到全局变量is_led_on和brightness中，后续通过这个update_led实现灯光强度更新
void process_cmd(char * cmd) {
  if (*cmd == '\0') {
    return;
  }
  char *token = strtok(cmd, " \t");
  if (token == NULL) {
    return;
  } else {
    if (strcmp(token, "ON") == 0) {
      // TODO在这里实现点灯逻辑
      brightness = 100;
      is_led_on = true;
      update_led();
      uart_write_bytes(UART_NUM_0, "OK\r\n", 4);
                
    }
    if (strcmp(token, "OFF") == 0) {
      // TODO在这里实现点灯逻辑
      brightness = 0;
      is_led_on = false;
      update_led();
      uart_write_bytes(UART_NUM_0, "OK\r\n", 4);
               
    }
    if (strcmp(token, "SPEED") == 0) {
      // TODO在这里实现点灯逻辑
      is_led_on = true;
      char*arg = strtok(NULL, " \t");
      if (arg == NULL) {
        ESP_LOGI(TAG, "无效输入，请加入等的亮度");
        return;
      }
      for (char *p = arg; *p; p++) {
        if (!isdigit((unsigned char)*p)) {
            ESP_LOGI(TAG,"ERROR: Invalid number");
            return;
        } else {
          int num = atoi(arg);
          if (num < 0 || num > 100) {
            ESP_LOGI(TAG, "ERROR: Invalid number");
            return;
          } else {
            brightness = num;
            uart_write_bytes(UART_NUM_0, "OK\r\n", 4);
            update_led();
          }
        }
      }
      
    }
    

  }
}

void ledc_init(void){
    int ch;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 4000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    uint8_t *cmd_buf = (uint8_t *)malloc(RD_BUF_SIZE);
    assert(dtmp && cmd_buf);
    int idx = 0;   // ← 跨事件保留的指令缓冲索引
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type) {
            case UART_DATA:
                int len = uart_read_bytes(EX_UART_NUM, dtmp,event.size < RD_BUF_SIZE ? event.size : RD_BUF_SIZE,
                                          pdMS_TO_TICKS(10));
                for (int i = 0; i < len; i++) {
                    uint8_t c = dtmp[i];

                    if (c == '\r' || c == '\n') {
                        // 遇到行尾，且缓冲里有内容 → 一条完整指令
                        if (idx > 0) {
                            cmd_buf[idx] = '\0';
                            process_cmd((char *)cmd_buf);
                            idx = 0;   // 清空，准备下一条
                        }
                        // 如果 idx == 0，是连续的空行/重复的 \r\n，忽略
                    } else if (idx < RD_BUF_SIZE - 1) {
                        cmd_buf[idx++] = c;
                    } else {
                        // 缓冲满，丢弃整条并报错
                        idx = 0;
                        uart_write_bytes(EX_UART_NUM, "ERROR: Command too long\r\n", 25);
                    }
                }
                break;
            
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                // int len = uart_read_bytes(EX_UART_NUM, dtmp, event.size,
                //                           portMAX_DELAY);
                
                // for (int i = 0; i < len; i++) {
                //   if (dtmp[i] == '\r' || dtmp[i] == '\n') {
                //     cmd_buf[i] = '\0';
                //   } else {
                //     cmd_buf[i] = dtmp[i];
                //   }
                // }
                // process_cmd((char *)cmd_buf);
                
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                // uart_flush_input(EX_UART_NUM);
                // xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(EX_UART_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(EX_UART_NUM);
                } else {
                    uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    free(cmd_buf);
    dtmp = NULL;
    cmd_buf =NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{

    esp_log_level_set(TAG, ESP_LOG_INFO);
    //这个地方进行ledc的初始化
    ledc_init();       // 初始化 LEDC
    update_led();      // 初始状态：灭
    ESP_LOGI("LEDC", "LEDC初始化成功");

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, GPIO_NUM_43, GPIO_NUM_44,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    uart_write_bytes(UART_NUM_0, "HELLO\r\n", 7);    
    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);

}
