#include "teach_drive.h"
#include "teach_course.h"
#include "teach_storage.h"
#include "board_drive.h"
#include "bluetooth_status.h"
#include "motion.h"
#include "motor.h"
#include "navigation_config.h"
#include "software_uart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRAIGHT_PWM 280
#define PIVOT_PWM 230
#define TAP_TURN_TENTHS 70
#define HOLD_TIMEOUT_MS 300u
#define SETTLE_MS 180u
#define STALL_MS 750u
#define MOVE_LIMIT_MS 20000u
#define E2_LIMIT 30000
#define TURN_LIMIT_TENTHS 7200
#define DEG_TO_RAD 0.01745329252f

static TeachRoute route;
static uint32_t saved_id;
static bool recording, replaying, save_pending, pose_known, at_start;
static bool course_playback;
static unsigned replay_index;
static uint8_t active, pending;
static uint8_t hold_key;
static bool hold_repeated;
static uint32_t hold_received_at;
static bool settling;
static uint32_t motion_started, stopped_at, progress_at;
static int progress_mark, cutoff;
static int16_t last_encoder;
static int32_t e2;
static float x_mm, y_mm, start_yaw, move_yaw, previous_yaw;
static float course_yaw;
static bool gyro_was_ready;
static const char *notice;
static const char *fault_notice;
static char saved_notice[48];
static void TrackPose(void);

static bool IsTurn(uint8_t direction)
{
    return direction == 'L' || direction == 'R';
}

static int DirectionSign(uint8_t direction)
{
    return direction == 'F' || direction == 'L' ? 1 : -1;
}

static bool GyroReady(void)
{
    return Motion_IsMPUReady() && motion_diag.state == 4;
}

static void ResetPose(void)
{
    x_mm = navigation_config.start_x_mm;
    y_mm = navigation_config.start_y_mm;
    start_yaw = previous_yaw = Motion_GetAngle();
    pose_known = at_start = true;
}

static int Progress(void)
{
    if (IsTurn(active))
        return (int)((Motion_GetAngle() - move_yaw) * 10.0f) * DirectionSign(active);
    return abs((int)e2);
}

static void StopMove(uint32_t now)
{
    BoardDrive_Stop();
    if (active && !settling) {
        if (pose_known && GyroReady())
            TrackPose();
        cutoff = Progress();
        stopped_at = now;
        settling = true;
    }
}

static void Fault(const char *reason, uint32_t now)
{
    StopMove(now);
    pending = 0;
    hold_key = 0;
    recording = replaying = save_pending = false;
    notice = fault_notice = reason;
}

static bool BeginMove(uint8_t direction, uint32_t now)
{
    if (!GyroReady()) {
        Fault("LOI_MPU_CAN_DAT_LAI", now);
        return false;
    }
    if (!pose_known) {
        notice = "DAT_XE_VE_DAU_GUI_Y_HOAC_Z";
        return false;
    }
    if (recording && route.count >= TEACH_MAX_SEGMENTS) {
        notice = "DAY_DUONG_GUI_V";
        return false;
    }
    if (!IsTurn(direction)) {
        bool straight_ready = replaying && course_playback ?
            Motion_BeginStraightHeading(course_yaw) : Motion_BeginStraight();
        if (!straight_ready) {
            Fault("LOI_MPU_CAN_DAT_LAI", now);
            return false;
        }
    }
    fault_notice = NULL;
    active = direction;
    at_start = false;
    e2 = cutoff = progress_mark = 0;
    last_encoder = Encoder_GetCount2();
    move_yaw = previous_yaw = Motion_GetAngle();
    motion_started = progress_at = now;
    return true;
}

static bool WithinField(float yaw)
{
    float rad = (navigation_config.start_heading_deg + yaw - start_yaw) * DEG_TO_RAD;
    float cos_yaw = fabsf(cosf(rad)), sin_yaw = fabsf(sinf(rad));
    float dx = (navigation_config.length_mm * cos_yaw + navigation_config.width_mm * sin_yaw) * 0.5f;
    float dy = (navigation_config.length_mm * sin_yaw + navigation_config.width_mm * cos_yaw) * 0.5f;
    float left = x_mm - dx, right = x_mm + dx, bottom = y_mm - dy, top = y_mm + dy;
    /* Openings at the start and finish lines allow the entire body past Y=0. */
    return left >= 0 && right <= 1800 && top <= 1200 && bottom >= -180 &&
           (right <= 600 || left >= 1200 || bottom >= 600);
}

static void TrackPose(void)
{
    int16_t encoder = Encoder_GetCount2();
    int old_e2 = (int)e2;
    e2 += (int16_t)(encoder - last_encoder);
    last_encoder = encoder;
    float yaw = Motion_GetAngle();
    if (!IsTurn(active)) {
        /* Use the commanded travel sign and actual change in E2 magnitude,
         * matching the distance convention of Motion. */
        float rad = (navigation_config.start_heading_deg +
                     (yaw + previous_yaw) * 0.5f - start_yaw) * DEG_TO_RAD;
        static const float scale = MOTION_FIELD_MM_PER_COUNT;
        float mm = (abs((int)e2) - abs(old_e2)) * scale * DirectionSign(active);
        x_mm += cosf(rad) * mm;
        y_mm += sinf(rad) * mm;
    }
    previous_yaw = yaw;
}

static bool HoldFresh(uint32_t now)
{
    return hold_repeated && now - hold_received_at < HOLD_TIMEOUT_MS;
}

static void BeginReplaySegment(uint32_t now)
{
    TeachSegment segment = route.segments[replay_index];
    if (course_playback && IsTurn(segment.direction))
        course_yaw += DirectionSign(segment.direction) * segment.target * 0.1f;
    BeginMove(segment.direction, now);
}

static void FinishMove(uint32_t now)
{
    uint8_t finished = active;
    active = 0;
    settling = false;
    if (recording && cutoff > 0) {
        route.segments[route.count++] = (TeachSegment){(uint16_t)cutoff, finished, 0};
        notice = route.count == TEACH_MAX_SEGMENTS ? "DAY_DUONG_GUI_V" : "DA_GHI_DOAN";
    }
    if (replaying) {
        if (++replay_index == route.count) {
            replaying = false;
            notice = "PHAT_LAI_XONG";
        } else {
            BeginReplaySegment(now);
        }
    } else if (pending) {
        uint8_t next = pending;
        pending = 0;
        if (!(next == hold_key && hold_repeated) || HoldFresh(now))
            BeginMove(next, now);
    }
}

static void UpdateMove(uint32_t now)
{
    if (!active)
        return;
    if (!GyroReady()) {
        pose_known = false;
        Fault("LOI_MPU_CAN_DAT_LAI", now);
    } else if (pose_known) {
        TrackPose();
        if (!WithinField(Motion_GetAngle()))
            Fault("GIOI_HAN_SAN", now);
    }
    if (settling) {
        if (now - stopped_at >= SETTLE_MS)
            FinishMove(now);
        return;
    }

    int progress = Progress();
    if (replaying) {
        bool reached = course_playback && IsTurn(active) ?
            (course_yaw - Motion_GetAngle()) * DirectionSign(active) <= 0.5f :
            progress >= route.segments[replay_index].target;
        if (reached) {
            StopMove(now);
            return;
        }
    }
    if (!replaying) {
        bool held = active == hold_key && hold_repeated;
        if (((held || active == 'F') && !HoldFresh(now)) ||
            (IsTurn(active) && !held && progress >= TAP_TURN_TENTHS)) {
            StopMove(now);
            return;
        }
    }
    if (now - motion_started >= MOVE_LIMIT_MS ||
        progress >= (IsTurn(active) ? TURN_LIMIT_TENTHS : E2_LIMIT)) {
        Fault("GIOI_HAN_DOAN", now);
        return;
    }
    if (progress >= progress_mark + 2) {
        progress_mark = progress;
        progress_at = now;
    }
    if (now - progress_at >= STALL_MS) {
        Fault(IsTurn(active) ? "LOI_TIEN_DO_GOC" : "LOI_TIEN_DO_E2", now);
        return;
    }
    if (IsTurn(active)) {
        int16_t pwm = PIVOT_PWM * DirectionSign(active);
        BoardDrive_Set(-pwm, pwm);
    } else if (!Motion_DriveStraight(DirectionSign(active), STRAIGHT_PWM)) {
        Fault("LOI_MPU_CAN_DAT_LAI", now);
    }
}

static void Command(uint8_t command, uint32_t now)
{
    if (command == 'x' || command == 'X' || command == ' ') {
        StopMove(now);
        pending = 0;
        hold_key = 0;
        replaying = save_pending = false;
        notice = "DUNG";
        return;
    }
    if (command == 'Y' || command == 'Z') {
        hold_key = 0;
        if (active) {
            StopMove(now);
            pending = 0;
            replaying = save_pending = false;
            notice = "DA_DUNG_GUI_LAI_LENH";
        } else if (!GyroReady()) {
            notice = "CHO_MPU";
        } else if (command == 'Z' && recording) {
            notice = "DANG_GHI";
        } else {
            ResetPose();
            route.count = 0;
            fault_notice = NULL;
            recording = command == 'Z';
            notice = recording ? "BAT_DAU_GHI" : "RESET_PHIEN";
        }
        return;
    }
    if (command == 'V' && recording) {
        StopMove(now);
        pending = 0;
        hold_key = 0;
        save_pending = true;
        return;
    }
    if (command == 'U' || command == 'P' || command == 'J') {
        if (save_pending) {
            notice = "DANG_LUU";
        } else if (recording) {
            notice = "DANG_GHI_GUI_V";
        } else if (active) {
            return;
        } else if (!GyroReady()) {
            notice = "CHO_MPU";
        } else if (command == 'P' && (!at_start || !pose_known)) {
            notice = "DAT_XE_VE_DAU_GUI_Y";
        } else if (command != 'J' && !(saved_id = TeachStorage_Load(&route))) {
            notice = "CHUA_CO_DUONG_DA_LUU";
        } else {
            hold_key = 0;
            course_playback = command == 'J';
            if (command != 'P')
                ResetPose();
            if (course_playback) {
                route.count = sizeof(teach_course) / sizeof(teach_course[0]);
                memcpy(route.segments, teach_course, sizeof(teach_course));
                course_yaw = start_yaw;
            }
            replay_index = 0;
            replaying = true;
            notice = course_playback ? "PHAT_TUYEN_T14" : "PHAT_LAI";
            BeginReplaySegment(now);
        }
        return;
    }
    if ((command != 'F' && command != 'B' && !IsTurn(command)) || replaying || save_pending)
        return;
    if (command != 'B') {
        hold_repeated = command == hold_key && now - hold_received_at < HOLD_TIMEOUT_MS;
        hold_key = command;
        hold_received_at = now;
        if (command == 'F' && !hold_repeated) {
            if (active)
                StopMove(now);
            pending = 0;
            notice = "GIU_F";
            return;
        }
    } else {
        hold_key = 0;
    }
    if (!active) {
        BeginMove(command, now);
    } else if (settling || command != active) {
        StopMove(now);
        pending = command;
    }
}

static void SaveIfStopped(void)
{
    if (active || !save_pending)
        return;
    save_pending = false;
    if (TeachStorage_Save(&route, &saved_id)) {
        recording = false;
        snprintf(saved_notice, sizeof(saved_notice), "DA_LUU:ID%lu:SEG%u",
                 (unsigned long)saved_id, (unsigned)route.count);
        notice = saved_notice;
    } else {
        notice = route.count ? "LOI_LUU_THU_LAI_V" : "CHUA_CO_DOAN";
    }
}

_Noreturn void TeachDrive_Run(I2C_HandleTypeDef *bus)
{
    saved_id = TeachStorage_Load(&route);
    Motion_Init(bus);
    static const char ranges_off[5][8] = {"---", "---", "---", "---", "---"};
    char line[BLUETOOTH_STATUS_SIZE] = {0};
    unsigned pos = 0;
    uint32_t last_report = HAL_GetTick() - BLUETOOTH_STATUS_INTERVAL_MS;
    for (;;) {
        uint8_t command;
        if (SoftUART_ReadByte(&command))
            Command(command, HAL_GetTick());
        Motion_Update(STRAIGHT_PWM);
        uint32_t now = HAL_GetTick();
        /* Motion invalidates yaw after a long sample gap or a read fault. */
        bool ready = GyroReady();
        if (!ready) {
            pose_known = at_start = false;
        } else if (!gyro_was_ready) {
            notice = "MPU_OK_DAT_XE_GUI_Y_HOAC_Z";
        }
        gyro_was_ready = ready;
        UpdateMove(now);
        SaveIfStopped();
        if (!line[pos] && now - last_report >= BLUETOOTH_STATUS_INTERVAL_MS) {
            char action[12];
            snprintf(action, sizeof(action), "%s:%c",
                     recording ? "GHI" : replaying ? "PHAT_LAI" : "DUNG",
                     active && !settling ? active : '-');
            BluetoothStatus_Format(line, sizeof(line), ready, Motion_GetAngle(), ranges_off,
                action, notice ? notice : !ready ? "CHO_MPU" : fault_notice,
                Encoder_GetCount1(), Encoder_GetCount2());
            notice = NULL;
            last_report = now;
            pos = 0;
        }
        if (line[pos])
            SoftUART_TransmitByte((uint8_t)line[pos++]);
        HAL_Delay(1);
    }
}
