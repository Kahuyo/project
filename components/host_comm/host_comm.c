#include "host_comm.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "host_comm";
static int g_uart_num = -1;
static needle_motor_t *g_nm = NULL;
static screw_motor_t *g_sm = NULL;

void host_comm_init(int uart_num, needle_motor_t *nm, screw_motor_t *sm){
    g_uart_num = uart_num;
    g_nm = nm;
    g_sm = sm;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(g_uart_num, &uart_config);
    uart_driver_install(g_uart_num, 2048, 2048, 0, NULL, 0);
}

static void handle_line(const char *ln){
    // simple ASCII commands:
    // START NEEDLE <freq> <amp>
    // STOP NEEDLE
    // START SCREW <freq> <amp>
    // STOP SCREW
    // SET KP <val>
    char cmd[64];
    int ret = sscanf(ln, "%63s", cmd);
    if(ret <= 0) return;

    if(strcmp(cmd, "START") == 0){
        char target[32];
        if(sscanf(ln, "START %31s", target) == 1){
            if(strcmp(target, "NEEDLE") == 0){
                float f=1.0, a=5.0;
                if(sscanf(ln, "START NEEDLE %f %f", &f, &a) >= 1){
                    needle_motor_start_micro(g_nm, f, a);
                    uart_write_bytes(g_uart_num, "ACK START NEEDLE\n", 17);
                }
            } else if(strcmp(target, "SCREW") == 0){
                float f=2.0, a=5.0;
                if(sscanf(ln, "START SCREW %f %f", &f, &a) >= 1){
                    screw_motor_micro_oscillate(g_sm, f, a);
                    uart_write_bytes(g_uart_num, "ACK START SCREW\n", 16);
                }
            }
        }
    } else if(strcmp(cmd, "STOP") == 0){
        char target[32];
        if(sscanf(ln, "STOP %31s", target) == 1){
            if(strcmp(target, "NEEDLE") == 0){
                needle_motor_stop_micro(g_nm);
                uart_write_bytes(g_uart_num, "ACK STOP NEEDLE\n", 16);
            } else if(strcmp(target, "SCREW") == 0){
                screw_motor_stop_micro(g_sm);
                uart_write_bytes(g_uart_num, "ACK STOP SCREW\n", 15);
            }
        }
    } else if(strcmp(cmd, "SET") == 0){
        char var[32];
        if(sscanf(ln, "SET %31s", var) == 1){
            if(strcmp(var, "KP") == 0){
                float v;
                if(sscanf(ln, "SET KP %f", &v) == 1){
                    g_nm->kp = v;
                    uart_write_bytes(g_uart_num, "ACK SET KP\n", 11);
                }
            }
        }
    } else if(strcmp(cmd, "STATUS") == 0){
        char buf[128];
        int n = snprintf(buf, sizeof(buf), "STATUS needle_center=%.2f\n", g_nm->center);
        uart_write_bytes(g_uart_num, buf, n);
    } else {
        uart_write_bytes(g_uart_num, "UNKNOWN\n", 8);
    }
}

static void host_comm_task(void *arg){
    uint8_t *buf = malloc(1024);
    int idx = 0;
    while(true){
        int len = uart_read_bytes(g_uart_num, buf+idx, 1, pdMS_TO_TICKS(200));
        if(len > 0){
            if(buf[idx] == '\n' || buf[idx] == '\r'){
                buf[idx] = 0;
                if(idx > 0) handle_line((char*)buf);
                idx = 0;
            } else {
                idx++;
                if(idx >= 1023) idx = 0;
            }
        }
    }
    free(buf);
}

void host_comm_start(void){
    xTaskCreate(host_comm_task, "host_comm", 4096, NULL, 5, NULL);
}
