#ifndef BLUETOOTH_STATUS_H
#define BLUETOOTH_STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include "text_format.h"

#define BLUETOOTH_STATUS_INTERVAL_MS 250u
#define BLUETOOTH_STATUS_SIZE 190u

static inline void BluetoothStatus_Format(char *buffer, size_t size, bool gyro_ready,
    float yaw, const char ranges[5][8], const char *action, const char *notice, int e1, int e2)
{
    if (!buffer || size < 2) {
        if (buffer && size) buffer[0] = '\0';
        return;
    }
    TextWriter text;
    Text_Init(&text, buffer, size);
    Text_String(&text, "Goc:");
    if (gyro_ready) {
        int32_t tenth = (int32_t)(yaw * 10.0f);
        uint32_t magnitude = tenth < 0 ? 0u - (uint32_t)tenth : (uint32_t)tenth;
        if (tenth < 0) Text_Char(&text, '-');
        Text_UInt(&text, magnitude / 10u);
        Text_Char(&text, '.');
        Text_UInt(&text, magnitude % 10u);
    } else {
        Text_String(&text, "---");
    }
    Text_String(&text, " do | ToF:HT=");
    Text_String(&text, ranges[0]);
    Text_String(&text, ",TL=");
    Text_String(&text, ranges[1]);
    Text_String(&text, ",G=");
    Text_String(&text, ranges[2]);
    Text_String(&text, ",TR=");
    Text_String(&text, ranges[3]);
    Text_String(&text, ",HP=");
    Text_String(&text, ranges[4]);
    Text_String(&text, " mm | Lenh:");
    Text_String(&text, action);
    if (notice) {
        Text_Char(&text, ':');
        Text_String(&text, notice);
    }
    Text_String(&text, " | E1:");
    Text_Int(&text, e1);
    Text_String(&text, ",E2:");
    Text_Int(&text, e2);
    /* Cần đủ 5 byte cho "*XX\r\n"; thiếu chỗ thì bỏ cả khung như bản snprintf. */
    if (text.length + 5 >= size) {
        buffer[0] = '\0';
        return;
    }
    uint8_t crc = 0;
    for (size_t i = 0; i < text.length; ++i) {
        crc ^= (uint8_t)buffer[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    TextWriter tail;
    Text_Init(&tail, text.cursor, size - text.length);
    Text_Char(&tail, '*');
    Text_HexByte(&tail, crc);
    Text_Char(&tail, '\r');
    Text_Char(&tail, '\n');
    Text_Finish(&tail);
}

#endif

