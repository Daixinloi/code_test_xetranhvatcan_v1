#ifndef TEXT_FORMAT_H
#define TEXT_FORMAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Bộ ghép chuỗi nhỏ cho telemetry. Firmware chỉ cần in số nguyên, dấu phẩy
 * động và một ký tự hex, nên không cần kéo cả snprintf/newlib stdio vào flash.
 * Chuỗi tạo ra giống hệt snprintf("%u"), ("%d"), ("%02X") trong các trường hợp
 * dùng ở đây, kể cả số âm và số 0. */
typedef struct {
    char *cursor;   /* Vị trí ghi kế tiếp, không vượt quá limit. */
    char *limit;    /* Con trỏ tới byte cuối cùng dành cho ký tự kết thúc. */
    size_t length;  /* Độ dài thật, kể cả phần bị cắt khi buffer quá nhỏ. */
} TextWriter;

static inline void Text_Init(TextWriter *writer, char *buffer, size_t size)
{
    writer->cursor = buffer;
    writer->limit = size ? buffer + size - 1 : buffer;
    writer->length = 0;
}

static inline void Text_Char(TextWriter *writer, char value)
{
    if (writer->cursor < writer->limit) *writer->cursor++ = value;
    ++writer->length;
}

static inline void Text_String(TextWriter *writer, const char *text)
{
    while (*text) Text_Char(writer, *text++);
}

static inline void Text_UInt(TextWriter *writer, uint32_t value)
{
    char digits[10];
    unsigned count = 0;
    do {
        digits[count++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value);
    while (count) Text_Char(writer, digits[--count]);
}

static inline void Text_Int(TextWriter *writer, int32_t value)
{
    if (value < 0) {
        Text_Char(writer, '-');
        Text_UInt(writer, 0u - (uint32_t)value);
    } else {
        Text_UInt(writer, (uint32_t)value);
    }
}

static inline void Text_HexByte(TextWriter *writer, uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    Text_Char(writer, digits[(value >> 4) & 0x0Fu]);
    Text_Char(writer, digits[value & 0x0Fu]);
}

/* Kết thúc chuỗi tại vị trí hiện tại (buffer luôn có chỗ cho ký tự NUL). */
static inline void Text_Finish(TextWriter *writer)
{
    *writer->cursor = '\0';
}

#endif
