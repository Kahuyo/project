#include "vesc_commands.h"
#include "vesc_protocol.h"
#include <string.h>
#include <stdint.h>

#define COMM_SET_POS 9
#define COMM_SET_RPM 8
#define COMM_SET_DUTY 5
#define COMM_SET_DETECT 11

static size_t pack_and_write(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_sz){
    return pack_short_frame(payload, payload_len, out, out_sz);
}

size_t cmd_set_pos_frame(double pos_deg, uint8_t *out, size_t out_sz){
    uint8_t payload[5];
    payload[0] = COMM_SET_POS;
    int32_t val = (int32_t)(pos_deg * 1000000.0);
    payload[1] = (val >> 24) & 0xFF;
    payload[2] = (val >> 16) & 0xFF;
    payload[3] = (val >> 8) & 0xFF;
    payload[4] = (val) & 0xFF;
    return pack_and_write(payload, 5, out, out_sz);
}

size_t cmd_set_rpm_frame(int rpm, uint8_t *out, size_t out_sz){
    uint8_t payload[5];
    payload[0] = COMM_SET_RPM;
    int32_t val = rpm;
    payload[1] = (val >> 24) & 0xFF;
    payload[2] = (val >> 16) & 0xFF;
    payload[3] = (val >> 8) & 0xFF;
    payload[4] = (val) & 0xFF;
    return pack_and_write(payload, 5, out, out_sz);
}

size_t cmd_set_duty_frame(double duty, uint8_t *out, size_t out_sz){
    uint8_t payload[5];
    payload[0] = COMM_SET_DUTY;
    int32_t val = (int32_t)(duty * 100000.0);
    payload[1] = (val >> 24) & 0xFF;
    payload[2] = (val >> 16) & 0xFF;
    payload[3] = (val >> 8) & 0xFF;
    payload[4] = (val) & 0xFF;
    return pack_and_write(payload, 5, out, out_sz);
}

size_t cmd_get_rotor_position_frame(uint8_t *out, size_t out_sz){
    uint8_t payload[2];
    payload[0] = COMM_SET_DETECT;
    payload[1] = 3; // DISP_POS_MODE_ENCODER
    return pack_and_write(payload, 2, out, out_sz);
}
