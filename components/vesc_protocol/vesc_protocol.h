#pragma once
#include <stdint.h>
#include <stddef.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len);
size_t pack_short_frame(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_sz);
