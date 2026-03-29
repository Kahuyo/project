#pragma once

// ------------ UART selection (adjust to board wiring) -------------
#include "driver/uart.h"
#define VESC_NEEDLE_UART_NUM    UART_NUM_1
#define VESC_SCREW_UART_NUM     UART_NUM_2
#define HOST_UART_NUM           UART_NUM_0

// Default pins for ESP32-DevKitC (change if using different board)
#define NEEDLE_TX_PIN 17
#define NEEDLE_RX_PIN 16
#define SCREW_TX_PIN 18
#define SCREW_RX_PIN 19

#define NEEDLE_BAUD 115200
#define SCREW_BAUD  115200

// ---------- Needle params (from original Python config) -------------
#define NEEDLE_CENTER_DEG 180.5f
#define NEEDLE_USE_LIVE_CENTER 1
#define NEEDLE_FEEDBACK_LATENCY_S 0.03f
#define NEEDLE_MAX_VEL_DEG_S 360.0f
#define NEEDLE_MAX_ACC_DEG_S2 2000.0f
#define NEEDLE_OSC_DT 0.01f
#define NEEDLE_PRE_ROLL_SETTLE_S 0.5f
#define NEEDLE_VEL_FF_SCALE 0.60f
#define NEEDLE_FEEDBACK_VEL_ALPHA 0.25f

// control defaults (overrideable from host)
#define DEFAULT_KP 0.005f
#define DEFAULT_KD 0.0f
#define DEFAULT_FF 0.1f

// ---------- Screw params (partial) -------------
#define SCREW_LEAD_MM_PER_REV 2.114065f
#define SCREW_OSC_DT 0.01f
#define SCREW_OSC_KP_RPM_PER_MM 220.0f
#define SCREW_OSC_VEL_FF_GAIN 1.0f
#define SCREW_OSC_MIN_RPM 110.0f
#define SCREW_OSC_MAX_RPM 320.0f

// Logging config
#define LOG_PRINT_INTERVAL_S 1.0f
