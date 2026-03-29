#include "vesc_protocol.h"
#include <string.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len){
    uint16_t crc = 0;
    for(size_t i=0;i<len;i++){
        crc ^= ((uint16_t)data[i]) << 8;
        for(int j=0;j<8;j++){
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc & 0xFFFF;
}

size_t pack_short_frame(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_sz){
    // short frame: 0x02 LEN payload CRC(2) 0x03
    if(out_sz < payload_len + 5) return 0;
    size_t idx = 0;
    out[idx++] = 0x02;
    out[idx++] = (uint8_t)payload_len;
    memcpy(&out[idx], payload, payload_len); idx += payload_len;
    uint16_t crc = crc16_ccitt(payload, payload_len);
    out[idx++] = (crc >> 8) & 0xFF;
    out[idx++] = crc & 0xFF;
    out[idx++] = 0x03;
    return idx;
}
