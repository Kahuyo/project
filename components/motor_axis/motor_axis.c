#include "motor_axis.h"
#include "vesc_commands.h"
#include "vesc_protocol.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "vesc_motor";

void vesc_motor_init(vesc_motor_t *m, int uart_num, int baud, int tx_pin, int rx_pin){
    m->uart_num = uart_num;
    m->baud = baud;
    m->tx_pin = tx_pin;
    m->rx_pin = rx_pin;
    m->last_raw = NAN;
    m->continuous_deg = 0.0;
    m->max_step_deg = 120.0;
}

bool vesc_motor_open(vesc_motor_t *m){
    uart_config_t uart_config = {
        .baud_rate = m->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    if (uart_param_config(m->uart_num, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return false;
    }
    if (uart_driver_install(m->uart_num, 4096, 4096, 10, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return false;
    }
    // set pins for this UART (tx, rx, rts, cts)
    uart_set_pin(m->uart_num, m->tx_pin, m->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART %d opened at %d (TX=%d RX=%d)", m->uart_num, m->baud, m->tx_pin, m->rx_pin);
    return true;
}

void vesc_motor_close(vesc_motor_t *m){
    uart_driver_delete(m->uart_num);
}

void vesc_motor_set_pos_deg(vesc_motor_t *m, double deg){
    uint8_t buf[64];
    size_t len = cmd_set_pos_frame(deg, buf, sizeof(buf));
    if(len) uart_write_bytes(m->uart_num, (const char*)buf, len);
}

void vesc_motor_set_rpm(vesc_motor_t *m, int rpm){
    uint8_t buf[64];
    size_t len = cmd_set_rpm_frame(rpm, buf, sizeof(buf));
    if(len) uart_write_bytes(m->uart_num, (const char*)buf, len);
}

void vesc_motor_set_duty(vesc_motor_t *m, double duty){
    uint8_t buf[64];
    size_t len = cmd_set_duty_frame(duty, buf, sizeof(buf));
    if(len) uart_write_bytes(m->uart_num, (const char*)buf, len);
}

// basic decode for rotor position payload
static double decode_rotor_payload(const uint8_t *payload, size_t len){
    if(len < 5) return NAN;
    const uint8_t *raw = payload + 1;
    int32_t vi = ((int32_t)raw[0] << 24) | ((int32_t)raw[1] << 16) | ((int32_t)raw[2] << 8) | ((int32_t)raw[3]);
    double val = ((double)vi) / 100000.0;
    if(isfinite(val) && fabs(val) < 3600.0) return val;
    return NAN;
}

double vesc_motor_get_position_deg(vesc_motor_t *m, double timeout_s){
    uint8_t get_frame[16];
    size_t l = cmd_get_rotor_position_frame(get_frame, sizeof(get_frame));
    if(l) uart_write_bytes(m->uart_num, (const char*)get_frame, l);

    uint8_t buf[512];
    int timeout_ms = (int)(timeout_s * 1000.0);
    int len = uart_read_bytes(m->uart_num, buf, sizeof(buf), pdMS_TO_TICKS(timeout_ms));
    if(len <= 0) return NAN;

    // search for short frame
    for(int i=0;i<len;i++){
        if(buf[i] == 0x02 && i+1 < len){
            int payload_len = buf[i+1];
            if(i + 2 + payload_len + 3 <= len){
                uint8_t *payload = &buf[i+2];
                double raw = decode_rotor_payload(payload, payload_len+1);
                if(isnan(raw)) return NAN;
                double wrapped = fmod(raw, 360.0);
                if (wrapped < 0) wrapped += 360.0;
                if (isnan(m->last_raw)) {
                    m->last_raw = wrapped;
                    m->continuous_deg = wrapped;
                    return m->continuous_deg;
                }
                double delta = fmod((wrapped - m->last_raw + 180.0), 360.0) - 180.0;
                if (fabs(delta) > m->max_step_deg) {
                    m->last_raw = wrapped;
                    return m->continuous_deg;
                }
                m->last_raw = wrapped;
                m->continuous_deg += delta;
                return m->continuous_deg;
            }
        }
    }
    return NAN;
}
