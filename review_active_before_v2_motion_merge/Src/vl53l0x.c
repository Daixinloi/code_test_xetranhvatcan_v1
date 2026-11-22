#include "vl53l0x.h"

/* Các thanh ghi nội bộ của VL53L0X */
#define REG_SYSRANGE_START                              0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                      0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                      0x0B
#define REG_RESULT_INTERRUPT_STATUS                     0x13
#define REG_RESULT_RANGE_STATUS                         0x14
#define REG_I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define REG_IDENTIFICATION_MODEL_ID                     0xC0
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

/* Hàm đọc / ghi thanh ghi I2C */
static bool writeReg(VL53L0X_Dev *dev, uint8_t reg, uint8_t value)
{
    bool ok = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, dev->timeout) == HAL_OK;
    if (!ok) dev->io_failed = true;
    return ok;
}

static bool readReg(VL53L0X_Dev *dev, uint8_t reg, uint8_t *value)
{
    bool ok = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, value, 1, dev->timeout) == HAL_OK;
    if (!ok) dev->io_failed = true;
    return ok;
}

static bool readReg16Bit(VL53L0X_Dev *dev, uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, dev->timeout) != HAL_OK)
    {
        dev->io_failed = true;
        return false;
    }
    *value = ((uint16_t)buffer[0] << 8) | buffer[1];
    return true;
}

/* Register protocol follows ST DataInit/StaticInit and the reference-SPAD
 * sequence documented by Pololu's VL53L0X driver. These are factory NVM
 * reference settings, not a guessed distance offset or contact calibration.
 * https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp */
static void WriteSequence(VL53L0X_Dev *dev, const uint8_t sequence[][2], unsigned count)
{
    for (unsigned i = 0; i < count; ++i) writeReg(dev, sequence[i][0], sequence[i][1]);
}

static bool ConfigureReferenceSpads(VL53L0X_Dev *dev)
{
    const uint8_t enter[][2] = {{0x80,1},{0xFF,1},{0x00,0},{0xFF,6}};
    WriteSequence(dev, enter, sizeof(enter) / sizeof(enter[0]));
    uint8_t control = 0, info = 0, ready = 0;
    readReg(dev, 0x83, &control);
    writeReg(dev, 0x83, control | 4);
    const uint8_t request[][2] = {{0xFF,7},{0x81,1},{0x80,1},{0x94,0x6B},{0x83,0}};
    WriteSequence(dev, request, sizeof(request) / sizeof(request[0]));
    uint32_t started = HAL_GetTick();
    while (!dev->io_failed && !ready && HAL_GetTick() - started < dev->timeout) {
        readReg(dev, 0x83, &ready);
        if (!ready) HAL_Delay(1);
    }
    if (ready) {
        writeReg(dev, 0x83, 1);
        readReg(dev, 0x92, &info);
    }
    const uint8_t leave[][2] = {{0x81,0},{0xFF,6}};
    WriteSequence(dev, leave, sizeof(leave) / sizeof(leave[0]));
    readReg(dev, 0x83, &control);
    writeReg(dev, 0x83, control & (uint8_t)~4);
    const uint8_t restore[][2] = {{0xFF,1},{0x00,1},{0xFF,0},{0x80,0}};
    WriteSequence(dev, restore, sizeof(restore) / sizeof(restore[0]));
    if (!ready || dev->io_failed || !(info & 0x7F)) return false;

    uint8_t map[6];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->address, 0xB0, 1, map, 6, dev->timeout) != HAL_OK)
        return false;
    dev->reference_spads = info & 0x7F;
    unsigned first = (info & 0x80) ? 12 : 0, enabled = 0;
    for (unsigned bit = 0; bit < 48; ++bit) {
        uint8_t mask = (uint8_t)(1u << (bit % 8));
        if (bit < first || enabled >= dev->reference_spads) map[bit / 8] &= (uint8_t)~mask;
        else if (map[bit / 8] & mask) ++enabled;
    }
    const uint8_t reference[][2] = {{0xFF,1},{0x4F,0},{0x4E,0x2C},{0xFF,0},{0xB6,0xB4}};
    WriteSequence(dev, reference, sizeof(reference) / sizeof(reference[0]));
    return enabled == dev->reference_spads && !dev->io_failed &&
        HAL_I2C_Mem_Write(dev->hi2c, dev->address, 0xB0, 1, map, 6, dev->timeout) == HAL_OK;
}

/* Thực hiện một bước hiệu chuẩn tham chiếu đơn (VHV hoặc Phase Calibration) */
static bool performSingleRefCalibration(VL53L0X_Dev *dev, uint8_t vhv_init_byte)
{
    writeReg(dev, REG_SYSRANGE_START, 0x01 | vhv_init_byte);

    uint32_t start_tick = HAL_GetTick();
    uint8_t status = 0;
    while ((status & 0x07) == 0)
    {
        if ((HAL_GetTick() - start_tick) > dev->timeout)
        {
            return false;
        }
        readReg(dev, REG_RESULT_INTERRUPT_STATUS, &status);
    }

    writeReg(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(dev, REG_SYSRANGE_START, 0x00);
    return true;
}

/* Hiệu chuẩn VHV và Pha - Bước quan trọng nhất để laser SPAD không bị trả về 8190 / 8191 */
static bool performRefCalibration(VL53L0X_Dev *dev)
{
    // 1. VHV Calibration (0x40)
    writeReg(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(dev, 0x40))
    {
        return false;
    }

    // 2. Phase Calibration (0x00)
    writeReg(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(dev, 0x00))
    {
        return false;
    }

    // Khôi phục lại cấu hình chu kỳ đo hoàn chỉnh
    writeReg(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    return true;
}

/* Đổi địa chỉ I2C của cảm biến (new_address_7bit: ví dụ 0x30, 0x32, ...) */
bool VL53L0X_SetAddress(VL53L0X_Dev *dev, uint8_t new_address_7bit)
{
    /* Ghi địa chỉ mới vào thanh ghi 0x8A */
    if (!writeReg(dev, REG_I2C_SLAVE_DEVICE_ADDRESS, new_address_7bit & 0x7F))
    {
        return false;
    }
    /* Cập nhật lại địa chỉ 8-bit trong cấu trúc struct của thiết bị */
    dev->address = (new_address_7bit << 1);
    HAL_Delay(2);
    return true;
}

/* Khởi tạo cảm biến VL53L0X theo chuỗi nạp chuẩn Pololu / ST API */
bool VL53L0X_Init(VL53L0X_Dev *dev, I2C_HandleTypeDef *hi2c, uint8_t address_8bit)
{
    dev->hi2c = hi2c;
    dev->address = address_8bit;
    dev->timeout = 200; /* Startup calibration only; runtime returns to 5 ms. */
    dev->io_failed = false;
    dev->samples = 0;
    dev->raw_mm = 8190;
    dev->device_status = 255;
    dev->reference_spads = 0;

    /* 1. Kiểm tra Model ID của VL53L0X (phải là 0xEE) */
    uint8_t model_id = 0;
    if (!readReg(dev, REG_IDENTIFICATION_MODEL_ID, &model_id) || model_id != 0xEE)
    {
        return false;
    }

    /* 2. Cấu hình I/O Pad 2V8 */
    uint8_t vhv_val = 0;
    readReg(dev, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &vhv_val);
    writeReg(dev, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vhv_val | 0x01);

    /* 3. Cài đặt I2C standard mode */
    writeReg(dev, 0x88, 0x00);

    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    readReg(dev, 0x91, &dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Vô hiệu hóa SIGNAL_RATE_MCLK và DSS */
    uint8_t msrc = 0;
    readReg(dev, 0x60, &msrc);
    writeReg(dev, 0x60, msrc | 0x12);

    /* Cấu hình ngắt GPIO */
    writeReg(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    if (!ConfigureReferenceSpads(dev)) return false;

    /* Nạp bảng tuning chuẩn ST */
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x09, 0x00);
    writeReg(dev, 0x10, 0x00);
    writeReg(dev, 0x11, 0x00);
    writeReg(dev, 0x24, 0x01);
    writeReg(dev, 0x25, 0xFF);
    writeReg(dev, 0x75, 0x00);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x4E, 0x2C);
    writeReg(dev, 0x48, 0x00);
    writeReg(dev, 0x30, 0x20);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x30, 0x09);
    writeReg(dev, 0x54, 0x00);
    writeReg(dev, 0x31, 0x04);
    writeReg(dev, 0x32, 0x03);
    writeReg(dev, 0x40, 0x83);
    writeReg(dev, 0x46, 0x25);
    writeReg(dev, 0x60, 0x00);
    writeReg(dev, 0x27, 0x00);
    writeReg(dev, 0x50, 0x06);
    writeReg(dev, 0x51, 0x00);
    writeReg(dev, 0x52, 0x96);
    writeReg(dev, 0x56, 0x08);
    writeReg(dev, 0x57, 0x30);
    writeReg(dev, 0x61, 0x00);
    writeReg(dev, 0x62, 0x00);
    writeReg(dev, 0x64, 0x00);
    writeReg(dev, 0x65, 0x00);
    writeReg(dev, 0x66, 0xA0);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x22, 0x32);
    writeReg(dev, 0x47, 0x14);
    writeReg(dev, 0x49, 0xFF);
    writeReg(dev, 0x4A, 0x00);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x7A, 0x0A);
    writeReg(dev, 0x7B, 0x00);
    writeReg(dev, 0x78, 0x21);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x23, 0x34);
    writeReg(dev, 0x42, 0x00);
    writeReg(dev, 0x44, 0xFF);
    writeReg(dev, 0x45, 0x26);
    writeReg(dev, 0x46, 0x05);
    writeReg(dev, 0x40, 0x40);
    writeReg(dev, 0x0E, 0x06);
    writeReg(dev, 0x20, 0x1A);
    writeReg(dev, 0x43, 0x40);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x34, 0x03);
    writeReg(dev, 0x35, 0x44);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x31, 0x04);
    writeReg(dev, 0x4B, 0x09);
    writeReg(dev, 0x4C, 0x05);
    writeReg(dev, 0x4D, 0x04);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x44, 0x00);
    writeReg(dev, 0x45, 0x20);
    writeReg(dev, 0x47, 0x08);
    writeReg(dev, 0x48, 0x28);
    writeReg(dev, 0x67, 0x00);
    writeReg(dev, 0x70, 0x04);
    writeReg(dev, 0x71, 0x01);
    writeReg(dev, 0x72, 0xFE);
    writeReg(dev, 0x76, 0x00);
    writeReg(dev, 0x77, 0x00);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x0D, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0x01, 0xF8);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x8E, 0x01);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* 4. Thực hiện Reference Calibration (VHV & Pha) */
    uint8_t interrupt_config = 0;
    writeReg(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    readReg(dev, 0x84, &interrupt_config);
    writeReg(dev, 0x84, interrupt_config & (uint8_t)~0x10);
    writeReg(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (dev->io_failed || !performRefCalibration(dev)) return false;

    /* 5. Xóa ngắt ban đầu */
    writeReg(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return !dev->io_failed;
}

/* Bắt đầu chế độ đo liên tục (Continuous Back-to-Back Mode) */
bool VL53L0X_StartContinuous(VL53L0X_Dev *dev)
{
    dev->io_failed = false;
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Bắt đầu đo liên tục không ngừng */
    writeReg(dev, REG_SYSRANGE_START, 0x02);
    return !dev->io_failed;
}

/* Dừng chế độ đo liên tục */
void VL53L0X_StopContinuous(VL53L0X_Dev *dev)
{
    writeReg(dev, REG_SYSRANGE_START, 0x01);

    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, 0x00);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
}

/* Đọc khoảng cách ở chế độ Continuous (Non-blocking, trả về 0 nếu chưa có mẫu mới) */
bool VL53L0X_ReadMeasurement(VL53L0X_Dev *dev, uint16_t *millimeters)
{
    uint8_t status = 0;
    if (readReg(dev, REG_RESULT_INTERRUPT_STATUS, &status) && ((status & 0x07) != 0))
    {
        uint8_t result[12];
        if (HAL_I2C_Mem_Read(dev->hi2c, dev->address, REG_RESULT_RANGE_STATUS,
                             1, result, sizeof(result), dev->timeout) != HAL_OK) {
            dev->io_failed = true;
            return false;
        }
        dev->device_status = (result[0] >> 3) & 0x0F;
        dev->raw_mm = ((uint16_t)result[10] << 8) | result[11];
        dev->samples++;
        if (!writeReg(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;
        /* ST's hardware status 11 denotes an accepted measurement. Signal,
         * phase, or minimum-range failures must not look like valid millimeters. */
        *millimeters = dev->device_status == 11 && dev->raw_mm < 8000 ? dev->raw_mm : 8190;
        return true;
    }
    return false;
}

uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_Dev *dev)
{
    uint16_t mm;
    return VL53L0X_ReadMeasurement(dev, &mm) ? mm : 0;
}

/* Đọc khoảng cách (Single Shot) */
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev *dev)
{
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Bắt đầu đo 1 lần */
    writeReg(dev, REG_SYSRANGE_START, 0x01);

    /* Chờ start bit được xóa */
    uint32_t start_tick = HAL_GetTick();
    uint8_t sysrange_start = 0x01;
    while (sysrange_start & 0x01)
    {
        if ((HAL_GetTick() - start_tick) > dev->timeout)
        {
            return 8190;
        }
        readReg(dev, REG_SYSRANGE_START, &sysrange_start);
    }

    /* Chờ cờ ngắt báo có kết quả */
    start_tick = HAL_GetTick();
    uint8_t status = 0;
    while ((status & 0x07) == 0)
    {
        if ((HAL_GetTick() - start_tick) > dev->timeout)
        {
            return 8190;
        }
        readReg(dev, REG_RESULT_INTERRUPT_STATUS, &status);
    }

    uint16_t range_mm = 0;
    readReg16Bit(dev, REG_RESULT_RANGE_STATUS + 10, &range_mm);
    writeReg(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range_mm;
}
