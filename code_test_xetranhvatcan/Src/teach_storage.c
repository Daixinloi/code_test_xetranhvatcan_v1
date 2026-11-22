#include "teach_storage.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>

#define ROUTE_COMMITTED 0xA719u
#define ROUTE_VERSION 1u

typedef struct {
    uint16_t committed, version;
    uint32_t run_id, checksum;
    TeachRoute route;
} FlashRoute;

extern uint8_t _teach_flash_start[];
_Static_assert(FLASH_PAGE_SIZE == 1024u, "Teach storage requires 1 KiB flash pages");
_Static_assert(sizeof(FlashRoute) <= FLASH_PAGE_SIZE, "Teach route does not fit one page");

static const volatile FlashRoute *Slot(unsigned index)
{
    return (const volatile FlashRoute *)((uintptr_t)_teach_flash_start + index * FLASH_PAGE_SIZE);
}

static unsigned RouteBytes(uint16_t count)
{
    return offsetof(TeachRoute, segments) + count * sizeof(TeachSegment);
}

static uint32_t Checksum(const volatile TeachRoute *route, uint32_t run_id)
{
    const volatile uint8_t *bytes = (const volatile uint8_t *)route;
    unsigned size = RouteBytes(route->count);
    uint32_t hash = 2166136261u ^ run_id;
    for (unsigned i = 0; i < size; ++i)
        hash = (hash ^ bytes[i]) * 16777619u;
    return hash;
}

static bool Valid(const volatile FlashRoute *slot)
{
    if (slot->committed != ROUTE_COMMITTED || slot->version != ROUTE_VERSION ||
        !slot->run_id || !slot->route.count || slot->route.count > TEACH_MAX_SEGMENTS ||
        slot->checksum != Checksum(&slot->route, slot->run_id))
        return false;
    for (unsigned i = 0; i < slot->route.count; ++i) {
        TeachSegment segment = slot->route.segments[i];
        if (!segment.target || (segment.direction != 'F' && segment.direction != 'B' &&
                               segment.direction != 'L' && segment.direction != 'R'))
            return false;
    }
    return true;
}

static const volatile FlashRoute *Latest(void)
{
    const volatile FlashRoute *a = Slot(0), *b = Slot(1);
    if (!Valid(a))
        return Valid(b) ? b : NULL;
    return Valid(b) && (int32_t)(b->run_id - a->run_id) > 0 ? b : a;
}

uint32_t TeachStorage_Load(TeachRoute *route)
{
    const volatile FlashRoute *slot = Latest();
    route->count = 0;
    if (!slot)
        return 0;
    for (unsigned i = 0; i < slot->route.count; ++i)
        route->segments[i] = slot->route.segments[i];
    route->count = slot->route.count;
    return slot->run_id;
}

bool TeachStorage_Save(const TeachRoute *route, uint32_t *run_id)
{
    if (!route->count || route->count > TEACH_MAX_SEGMENTS)
        return false;
    const volatile FlashRoute *latest = Latest();
    const volatile FlashRoute *slot = Slot(latest == Slot(0) ? 1 : 0);
    uint32_t next_id = latest ? latest->run_id + 1u : 1u;
    if (!next_id)
        next_id = 1u;
    FlashRoute data = {
        .version = ROUTE_VERSION,
        .run_id = next_id,
        .checksum = Checksum(route, next_id),
        .route = *route
    };
    uint32_t address = (uint32_t)(uintptr_t)slot;
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES, .PageAddress = address, .NbPages = 1
    };
    uint32_t failed_page;
    if (HAL_FLASH_Unlock() != HAL_OK)
        return false;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    bool ok = HAL_FLASHEx_Erase(&erase, &failed_page) == HAL_OK;
    const uint8_t *bytes = (const uint8_t *)&data;
    unsigned end = offsetof(FlashRoute, route) + RouteBytes(route->count);
    for (unsigned i = sizeof(data.committed); ok && i < end; i += 2) {
        uint16_t halfword = bytes[i] | ((uint16_t)bytes[i + 1] << 8);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, halfword) == HAL_OK;
    }
    if (ok && slot->route.count == route->count && slot->run_id == next_id &&
        slot->version == ROUTE_VERSION && slot->checksum == Checksum(&slot->route, next_id)) {
        /* Commit only after the full payload is readable. The old slot survives. */
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, ROUTE_COMMITTED) == HAL_OK;
    } else {
        ok = false;
    }
    HAL_FLASH_Lock();
    if (!ok || !Valid(slot))
        return false;
    *run_id = next_id;
    return true;
}
