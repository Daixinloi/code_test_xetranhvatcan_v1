#include "bluetooth_status.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Khung telemetry được điện thoại và Tools/map_monitor/stlink_feed.py đọc.
 * Test này dựng lại đúng chuỗi bằng snprintf (chỉ có trên PC) rồi so sánh với
 * bộ ghép chuỗi dùng trong firmware, để đổi cách in không làm lệch dữ liệu. */
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static void Expected(char *out, size_t size, bool gyro_ready, float yaw,
    const char ranges[5][8], const char *action, const char *notice, int e1, int e2)
{
    char heading[24];
    int32_t tenth = (int32_t)(yaw * 10.0f);
    uint32_t magnitude = tenth < 0 ? (uint32_t)(-tenth) : (uint32_t)tenth;
    if (gyro_ready)
        snprintf(heading, sizeof(heading), "%s%lu.%lu", tenth < 0 ? "-" : "",
                 (unsigned long)(magnitude / 10), (unsigned long)(magnitude % 10));
    else
        strcpy(heading, "---");
    int length = snprintf(out, size,
        "Goc:%s do | ToF:HT=%s,TL=%s,G=%s,TR=%s,HP=%s mm | Lenh:%s%s%s | E1:%d,E2:%d",
        heading, ranges[0], ranges[1], ranges[2], ranges[3], ranges[4],
        action, notice ? ":" : "", notice ? notice : "", e1, e2);
    if (size < 2 || length < 0 || (size_t)length + 5 >= size) {
        /* Bản snprintf để khung rỗng khi không đủ chỗ cho CRC. */
        if (length >= 0) out[0] = '\0';
        return;
    }
    uint8_t crc = 0;
    for (int i = 0; i < length; ++i) {
        crc ^= (uint8_t)out[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    snprintf(out + length, size - (size_t)length, "*%02X\r\n", (unsigned)crc);
}

static void Compare(bool gyro_ready, float yaw, const char *r0, const char *r1,
    const char *r2, const char *r3, const char *r4, const char *action,
    const char *notice, int e1, int e2)
{
    const char *values[5] = {r0, r1, r2, r3, r4};
    char ranges[5][8];
    for (unsigned i = 0; i < 5; ++i) {
        memset(ranges[i], 0, sizeof(ranges[i]));
        strncpy(ranges[i], values[i], sizeof(ranges[i]) - 1);
    }
    char actual[BLUETOOTH_STATUS_SIZE], expected[BLUETOOTH_STATUS_SIZE];
    memset(actual, 0, sizeof(actual));
    memset(expected, 0, sizeof(expected));
    BluetoothStatus_Format(actual, sizeof(actual), gyro_ready, yaw, ranges, action, notice, e1, e2);
    Expected(expected, sizeof(expected), gyro_ready, yaw, ranges, action, notice, e1, e2);
    if (strcmp(actual, expected)) {
        fprintf(stderr, "MISMATCH yaw=%.3f action=%s notice=%s e1=%d e2=%d\n"
                        " actual: %s\n expect: %s\n", yaw, action,
                notice ? notice : "(null)", e1, e2, actual, expected);
        exit(1);
    }
}

int main(void)
{
    Compare(false, 0.0f, "---", "---", "---", "---", "---", "DUNG", NULL, 0, 0);
    Compare(true, 0.0f, "8190", "8191", "120", "35", "---", "TIEN", NULL, 0, 0);
    Compare(true, 1.26f, "45", "300", "1500", "92", "8190", "U-SANG-PHAI",
            "GIU-5CM", -12, 345);
    Compare(true, -1.26f, "1", "2", "3", "4", "5", "U-XUONG", "MAP_LOI_TOF_TL_G_TR",
            -32768, 32767);
    Compare(true, 359.99f, "10", "20", "30", "40", "50", "RE-P90", "I2C_PHUC_HOI",
            -2147483647 - 1, 2147483647);
    Compare(true, -0.04f, "0", "0", "0", "0", "0", "NE-T45", "", 1, -1);

    /* Không đủ chỗ cho CRC: cả hai bản phải trả về khung rỗng. */
    char buffer[40];
    char expected[40];
    char ranges[5][8] = {"100", "200", "300", "400", "500"};
    memset(buffer, 'x', sizeof(buffer));
    memset(expected, 'x', sizeof(expected));
    BluetoothStatus_Format(buffer, sizeof(buffer), true, 12.3f, ranges, "TIEN", "GIU-5CM", 1, 2);
    CHECK(buffer[0] == '\0');
    Expected(expected, sizeof(expected), true, 12.3f, ranges, "TIEN", "GIU-5CM", 1, 2);
    CHECK(strcmp(buffer, expected) == 0);

    /* Đúng ngưỡng: length + 5 == size thì bỏ, lớn hơn 1 byte thì giữ đủ. */
    char big[BLUETOOTH_STATUS_SIZE];
    BluetoothStatus_Format(big, sizeof(big), false, 0, ranges, "DUNG", NULL, 0, 0);
    size_t length = strlen(big) - 5;
    char tight[BLUETOOTH_STATUS_SIZE];
    BluetoothStatus_Format(tight, length + 5, false, 0, ranges, "DUNG", NULL, 0, 0);
    CHECK(tight[0] == '\0');
    char fits[BLUETOOTH_STATUS_SIZE];
    BluetoothStatus_Format(fits, length + 6, false, 0, ranges, "DUNG", NULL, 0, 0);
    CHECK(!strcmp(fits, big));

    puts("PASS: telemetry frame matches snprintf byte for byte, CRC and overflow unchanged");
    return 0;
}
