#include "screw_motor.h"
#include "config/include/config.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "screw";

static float deg_to_mm(float deg){
    return deg * SCREW_LEAD_MM_PER_REV / 360.0f;
}
static float mm_to_deg(float mm){
    return mm * 360.0f / SCREW_LEAD_MM_PER_REV;
}

void screw_motor_init(screw_motor_t *sm, int uart_num, int baud, int tx_pin, int rx_pin){
    vesc_motor_init(&sm->vesc, uart_num, baud, tx_pin, rx_pin);
    sm->micro_active = false;
    sm->micro_task = NULL;
}

bool screw_motor_open(screw_motor_t *sm){
    return vesc_motor_open(&sm->vesc);
}

typedef struct {
    screw_motor_t *sm;
    float freq;
    float amp;
} screw_task_arg_t;

static void screw_micro_task(void *arg){
    screw_task_arg_t *a = (screw_task_arg_t*)arg;
    screw_motor_t *sm = a->sm;
    float freq = a->freq;
    float amp_deg = a->amp;
    free(a);

    sm->micro_active = true;
    float amp_mm = deg_to_mm(amp_deg);

    float t = 0.0f;
    const TickType_t dt_ticks = pdMS_TO_TICKS((int)(SCREW_OSC_DT*1000.0));
    TickType_t last_wake = xTaskGetTickCount();
    float last_print = 0.0f;

    // read zero position
    double zero_deg_d = vesc_motor_get_position_deg(&sm->vesc, 0.05);
    float zero_deg = isnan(zero_deg_d) ? 0.0f : (float)zero_deg_d;

    while (sm->micro_active) {
        float phase = 2.0f * M_PI * freq * t;
        float target_mm = amp_mm * sinf(phase);
        float desired_vel_mm_s = amp_mm * 2.0f * M_PI * freq * cosf(phase);

        double pos_deg_d = vesc_motor_get_position_deg(&sm->vesc, 0.05);
        float current_mm = deg_to_mm(isnan(pos_deg_d) ? zero_deg : ((float)pos_deg_d - zero_deg));

        float err_mm = target_mm - current_mm;
        float rpm_ff = -SCREW_OSC_VEL_FF_GAIN * (desired_vel_mm_s * 60.0f / SCREW_LEAD_MM_PER_REV);
        float rpm_p = -SCREW_OSC_KP_RPM_PER_MM * err_mm;
        float rpm_cmd_f = rpm_ff + rpm_p;
        float rpm_cmd = fmaxf(-SCREW_OSC_MAX_RPM, fminf(SCREW_OSC_MAX_RPM, rpm_cmd_f));

        vesc_motor_set_rpm(&sm->vesc, (int)roundf(rpm_cmd));

        if (t - last_print > 0.5f) {
            ESP_LOGI(TAG, "t=%.2f tgt_mm=%.3f cur_mm=%.3f err_mm=%.3f rpm=%.1f", t, target_mm, current_mm, err_mm, rpm_cmd);
            last_print = t;
        }

        t += SCREW_OSC_DT;
        vTaskDelayUntil(&last_wake, dt_ticks);
    }

    vesc_motor_set_rpm(&sm->vesc, 0);
    sm->micro_active = false;
    vTaskDelete(NULL);
}

void screw_motor_micro_oscillate(screw_motor_t *sm, float freq, float amp_deg){
    if (sm->micro_active) return;
    screw_task_arg_t *arg = malloc(sizeof(screw_task_arg_t));
    arg->sm = sm;
    arg->freq = freq;
    arg->amp = amp_deg;
    xTaskCreate(screw_micro_task, "screw_micro", 4096, arg, configMAX_PRIORITIES-3, &sm->micro_task);
}

void screw_motor_stop_micro(screw_motor_t *sm){
    sm->micro_active = false;
}
