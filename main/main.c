#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "components/needle/needle_motor.h"
#include "components/screw/screw_motor.h"
#include "components/host_comm/host_comm.h"
#include "components/config/include/config.h"

static const char *TAG = "main_app";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Dual VESC controller starting");

    static needle_motor_t needle;
    static screw_motor_t screw;

    needle_motor_init(&needle, VESC_NEEDLE_UART_NUM, NEEDLE_BAUD, NEEDLE_TX_PIN, NEEDLE_RX_PIN);
    screw_motor_init(&screw, VESC_SCREW_UART_NUM, SCREW_BAUD, SCREW_TX_PIN, SCREW_RX_PIN);

    if (!needle_motor_open(&needle)) {
        ESP_LOGW(TAG, "needle open failed");
    }
    if (!screw_motor_open(&screw)) {
        ESP_LOGW(TAG, "screw open failed");
    }

    host_comm_init(HOST_UART_NUM, &needle, &screw);
    host_comm_start();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
