#ifndef BLUETOOTH_STATUS_H
#define BLUETOOTH_STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BLUETOOTH_STATUS_INTERVAL_MS 250u
#define BLUETOOTH_STATUS_SIZE 190u

static inline void BluetoothStatus_Format(char *buffer, size_t size, bool gyro_ready,
    float yaw, const char ranges[5][8], const char *action, const char *notice, int e1, int e2)
{
    char heading[24];
    int32_t tenth = (int32_t)(yaw * 10.0f);
    uint32_t magnitude = tenth < 0 ? (uint32_t)(-tenth) : (uint32_t)tenth;
    if (gyro_ready)
        snprintf(heading, sizeof(heading), "%s%lu.%lu", tenth < 0 ? "-" : "",
                 (unsigned long)(magnitude / 10), (unsigned long)(magnitude % 10));
    else
        strcpy(heading, "---");
    int length = snprintf(buffer, size,
        "Goc:%s do | ToF:HT=%s,TL=%s,G=%s,TR=%s,HP=%s mm | Lenh:%s%s%s | E1:%d,E2:%d",
        heading, ranges[0], ranges[1], ranges[2], ranges[3], ranges[4],
        action, notice ? ":" : "", notice ? notice : "", e1, e2);
    if (length < 0 || (size_t)length + 5 >= size) {
        buffer[0] = '\0';
        return;
    }
    uint8_t crc = 0;
    for (int i = 0; i < length; ++i) {
        crc ^= (uint8_t)buffer[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    snprintf(buffer + length, size - (size_t)length, "*%02X\r\n", (unsigned)crc);
}

#endif
