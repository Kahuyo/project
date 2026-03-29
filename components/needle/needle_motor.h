#pragma once
#include "motor_axis.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    vesc_motor_t vesc;
    float center;
    float last_target;
    float last_vel;
    bool micro_active;
    TaskHandle_t micro_task;
    float kp, kd, ff_scale;
} needle_motor_t;

void needle_motor_init(needle_motor_t *nm, int uart_num, int baud, int tx_pin, int rx_pin);
bool needle_motor_open(needle_motor_t *nm);
void needle_motor_start_micro(needle_motor_t *nm, float freq, float amp_deg);
void needle_motor_stop_micro(needle_motor_t *nm);
