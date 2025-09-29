#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_S1 GPIO_NUM_19
#define LED_S2 GPIO_NUM_4
#define LED_S3 GPIO_NUM_5
#define LED_R1 GPIO_NUM_18
#define LED_R2 GPIO_NUM_6
#define LED_R3 GPIO_NUM_7

static const char *TAG = "queue_proj";

// --- Cấu trúc dữ liệu ---
typedef struct {
    uint8_t sender_id;      // NEW: để biết gói này từ sender nào
    uint8_t payload[64];    // 512 bit = 64 byte
} ImageRadarData;

typedef struct {
    uint32_t sensor_data;   // 32 bit
} EnvSensorData;

// --- Queue ---
static QueueHandle_t queue_large; // Sender1 + Sender2 dùng chung
static QueueHandle_t queue_small; // Sender3 riêng

// --- LED blink ---
static void blink_led(gpio_num_t pin, int duration_ms) {
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(pin, 0);
}

// --- Sender1: Image ---
static void sender1_task(void *pvParameters) {
    ImageRadarData data;
    data.sender_id = 1;
    memset(data.payload, 0xAA, sizeof(data.payload));

    while (1) {
        if (xQueueSend(queue_large, &data, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGI(TAG, "Sender1 (Image) sent");
            blink_led(LED_S1, 200);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// --- Sender2: Radar ---
static void sender2_task(void *pvParameters) {
    ImageRadarData data;
    data.sender_id = 2;
    memset(data.payload, 0xBB, sizeof(data.payload));

    while (1) {
        if (xQueueSend(queue_large, &data, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGI(TAG, "Sender2 (Radar) sent");
            blink_led(LED_S2, 200);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- Sender3: Env ---
static void sender3_task(void *pvParameters) {
    EnvSensorData signal;
    signal.sensor_data = 0xDEADBEEF;

    while (1) {
        if (xQueueSend(queue_small, &signal, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGI(TAG, "Sender3 (Env) sent");
            blink_led(LED_S3, 200);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// --- Receiver chung cho queue_large ---
static void receiver_large_task(void *pvParameters) {
    ImageRadarData data;
    while (1) {
        if (xQueueReceive(queue_large, &data, pdMS_TO_TICKS(1000)) == pdPASS) {
            if (data.sender_id == 1) {
                ESP_LOGI(TAG, "Receiver got Image data (from Sender1)");
                blink_led(LED_R1, 500);
            } else if (data.sender_id == 2) {
                ESP_LOGI(TAG, "Receiver got Radar data (from Sender2)");
                blink_led(LED_R2, 500);
            }
        }
    }
}

// --- Receiver3 cho Env ---
static void receiver3_task(void *pvParameters) {
    EnvSensorData signal;
    while (1) {
        if (xQueueReceive(queue_small, &signal, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Receiver3 got Env data: 0x%08X", (unsigned int)signal.sensor_data);
            blink_led(LED_R3, 500);
        }
    }
}

// --- Monitor ---
static void monitor_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Queue large: used=%u free=%u | Queue small: used=%u free=%u",
                 uxQueueMessagesWaiting(queue_large),
                 uxQueueSpacesAvailable(queue_large),
                 uxQueueMessagesWaiting(queue_small),
                 uxQueueSpacesAvailable(queue_small));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- Main ---
void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<LED_S1) | (1ULL<<LED_S2) | (1ULL<<LED_S3) |
                        (1ULL<<LED_R1) | (1ULL<<LED_R2) | (1ULL<<LED_R3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    queue_large = xQueueCreate(5, sizeof(ImageRadarData));
    queue_small = xQueueCreate(3, sizeof(EnvSensorData));

    if (queue_large && queue_small) {
        xTaskCreate(sender1_task, "Sender1", 2048, NULL, 3, NULL);
        xTaskCreate(sender2_task, "Sender2", 2048, NULL, 2, NULL);
        xTaskCreate(sender3_task, "Sender3", 2048, NULL, 1, NULL);

        xTaskCreate(receiver_large_task, "ReceiverLarge", 2048, NULL, 3, NULL);
        xTaskCreate(receiver3_task, "Receiver3", 2048, NULL, 2, NULL);

        xTaskCreate(monitor_task, "Monitor", 2048, NULL, 1, NULL);
        ESP_LOGI(TAG, "All tasks created");
    } else {
        ESP_LOGE(TAG, "Queue creation failed");
    }
}