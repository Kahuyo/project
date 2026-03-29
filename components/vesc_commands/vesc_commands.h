#pragma once
#include <stddef.h>

size_t cmd_set_pos_frame(double pos_deg, uint8_t *out, size_t out_sz);
size_t cmd_set_rpm_frame(int rpm, uint8_t *out, size_t out_sz);
size_t cmd_set_duty_frame(double duty, uint8_t *out, size_t out_sz);
size_t cmd_get_rotor_position_frame(uint8_t *out, size_t out_sz); // toggles encoder output
