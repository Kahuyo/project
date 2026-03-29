#pragma once
#include <stdbool.h>

typedef struct {
    int uart_num;
    int baud;
    int tx_pin;
    int rx_pin;
    double last_raw;
    double continuous_deg;
    double max_step_deg;
} vesc_motor_t;

void vesc_motor_init(vesc_motor_t *m, int uart_num, int baud, int tx_pin, int rx_pin);
bool vesc_motor_open(vesc_motor_t *m);
void vesc_motor_close(vesc_motor_t *m);

void vesc_motor_set_pos_deg(vesc_motor_t *m, double deg);
void vesc_motor_set_rpm(vesc_motor_t *m, int rpm);
void vesc_motor_set_duty(vesc_motor_t *m, double duty);

double vesc_motor_get_position_deg(vesc_motor_t *m, double timeout_s);
