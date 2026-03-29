#pragma once
#include "motor_axis.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    vesc_motor_t vesc;
    bool micro_active;
    TaskHandle_t micro_task;
} screw_motor_t;

void screw_motor_init(screw_motor_t *sm, int uart_num, int baud, int tx_pin, int rx_pin);
bool screw_motor_open(screw_motor_t *sm);
void screw_motor_micro_oscillate(screw_motor_t *sm, float freq, float amp_deg);
void screw_motor_stop_micro(screw_motor_t *sm);
