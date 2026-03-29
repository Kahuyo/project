#include "needle_motor.h"
#include "config/include/config.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "needle";

void needle_motor_init(needle_motor_t *nm, int uart_num, int baud, int tx_pin, int rx_pin){
    vesc_motor_init(&nm->vesc, uart_num, baud, tx_pin, rx_pin);
    nm->center = NEEDLE_CENTER_DEG;
    nm->last_target = nm->center;
    nm->last_vel = 0.0f;
    nm->micro_active = false;
    nm->micro_task = NULL;
    nm->kp = DEFAULT_KP;
    nm->kd = DEFAULT_KD;
    nm->ff_scale = DEFAULT_FF;
}

bool needle_motor_open(needle_motor_t *nm){
    return vesc_motor_open(&nm->vesc);
}

static float _clamp_f(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static float _limit_step(needle_motor_t *nm, float target, float dt){
    float max_step = NEEDLE_MAX_VEL_DEG_S * dt;
    float pos_cmd = nm->last_target + fmaxf(-max_step, fminf(max_step, target - nm->last_target));
    float vel = (pos_cmd - nm->last_target) / fmaxf(dt, 1e-6f);
    float max_dv = NEEDLE_MAX_ACC_DEG_S2 * dt;
    vel = nm->last_vel + fmaxf(-max_dv, fminf(max_dv, vel - nm->last_vel));
    pos_cmd = nm->last_target + vel * dt;
    nm->last_vel = vel;
    nm->last_target = pos_cmd;
    return pos_cmd;
}

typedef struct {
    needle_motor_t *nm;
    float freq;
    float amp;
} needle_task_arg_t;

static void needle_micro_task(void *arg){
    needle_task_arg_t *a = (needle_task_arg_t*)arg;
    needle_motor_t *nm = a->nm;
    float freq = a->freq;
    float amp = a->amp;
    free(a);

    nm->micro_active = true;

    // try to refresh center by reading a few samples
    double pos = vesc_motor_get_position_deg(&nm->vesc, 0.05);
    if(!isnan(pos)){
        nm->center = (float)pos;
        nm->last_target = nm->center;
        nm->last_vel = 0.0f;
        ESP_LOGI(TAG, "center calibrated=%.2f", nm->center);
    } else {
        ESP_LOGW(TAG, "live center unavailable, fallback to configured %.2f", nm->center);
    }

    vTaskDelay(pdMS_TO_TICKS((int)(NEEDLE_PRE_ROLL_SETTLE_S*1000)));

    const TickType_t dt_ticks = pdMS_TO_TICKS((int)(NEEDLE_OSC_DT*1000.0));
    TickType_t last_wake = xTaskGetTickCount();
    float t = 0.0f;
    float last_print_t = 0.0f;

    while(nm->micro_active){
        float phase = 2.0f * M_PI * freq * t;
        float raw_target = nm->center + amp * sinf(phase);
        float desired_vel = amp * 2.0f * M_PI * freq * cosf(phase);

        float bias = 0.0f; // directional bias could be applied
        float target_biased = raw_target + bias;
        float pred_target = target_biased + desired_vel * nm->ff_scale * NEEDLE_FEEDBACK_LATENCY_S;

        double fb_d = vesc_motor_get_position_deg(&nm->vesc, 0.02);
        float fb = isnan(fb_d) ? nm->last_target : (float)fb_d;

        float err = pred_target - fb;
        float derr = 0.0f; // derivative could be estimated with state
        float corr = nm->kp * err + nm->kd * derr;
        float max_corr = amp * 0.6f;
        corr = _clamp_f(corr, -max_corr, max_corr);

        float cmd_target = target_biased + corr;
        float cmd = _limit_step(nm, cmd_target, NEEDLE_OSC_DT);

        vesc_motor_set_pos_deg(&nm->vesc, cmd);

        nm->last_target = cmd;

        if (t - last_print_t > 0.5f) {
            ESP_LOGI(TAG, "t=%.2f target=%.2f cmd=%.2f fb=%.2f err=%.3f", t, raw_target, cmd, fb, err);
            last_print_t = t;
        }

        t += NEEDLE_OSC_DT;
        vTaskDelayUntil(&last_wake, dt_ticks);
    }

    // smooth return to center
    for (int i = 0; i < 60; ++i) {
        float cmd = _limit_step(nm, nm->center, NEEDLE_OSC_DT);
        vesc_motor_set_pos_deg(&nm->vesc, cmd);
        vTaskDelay(pdMS_TO_TICKS((int)(NEEDLE_OSC_DT*1000.0)));
    }

    nm->micro_active = false;
    vTaskDelete(NULL);
}

void needle_motor_start_micro(needle_motor_t *nm, float freq, float amp_deg){
    if (nm->micro_active) return;
    needle_task_arg_t *arg = malloc(sizeof(needle_task_arg_t));
    arg->nm = nm;
    arg->freq = freq;
    arg->amp = amp_deg;
    xTaskCreate(needle_micro_task, "needle_micro", 4096, arg, configMAX_PRIORITIES-2, &nm->micro_task);
}

void needle_motor_stop_micro(needle_motor_t *nm){
    nm->micro_active = false;
}
