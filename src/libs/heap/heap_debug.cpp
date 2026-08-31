#include "heap/heap_debug.h"

#include <cstddef>
#include <cstring>
#include <limits>

namespace {

constexpr size_t alignment = portBYTE_ALIGNMENT;
constexpr uintptr_t alignment_mask = alignment - 1U;

struct HeapAreaHeader {
    void *next;
    size_t size;
};

constexpr size_t header_size = (sizeof(HeapAreaHeader) + alignment_mask) & ~alignment_mask;
constexpr size_t allocated_mask = size_t{1} << (std::numeric_limits<size_t>::digits - 1U);

static_assert(alignment == 8U, "heap layout walker requires 8-byte alignment");

bool visit_region(const HeapRegion_t &region, HeapAreaVisitor_t visitor,
                  void *context, HeapLayoutStats_t &stats)
{
    uintptr_t start = reinterpret_cast<uintptr_t>(region.pucStartAddress);
    size_t region_size = region.xSizeInBytes;

    if((start & alignment_mask) != 0U) {
        const uintptr_t aligned_start = (start + alignment_mask) & ~alignment_mask;
        const size_t alignment_loss = aligned_start - start;
        if(alignment_loss >= region_size) return false;
        start = aligned_start;
        region_size -= alignment_loss;
    }

    if(region_size < header_size || region_size > std::numeric_limits<uintptr_t>::max() - start)
        return false;

    const uintptr_t end_marker = (start + region_size - header_size) & ~alignment_mask;
    uintptr_t cursor = start;

    while(cursor < end_marker) {
        size_t encoded_size;
        std::memcpy(&encoded_size,
                    reinterpret_cast<const void *>(cursor + offsetof(HeapAreaHeader, size)),
                    sizeof(encoded_size));

        const bool allocated = (encoded_size & allocated_mask) != 0U;
        const size_t area_size = encoded_size & ~allocated_mask;
        const size_t remaining = end_marker - cursor;
        if(area_size < header_size || (area_size & alignment_mask) != 0U || area_size > remaining)
            return false;

        const HeapAreaInfo_t area = {
            reinterpret_cast<const uint8_t *>(cursor), area_size, allocated,
        };
        if(allocated) {
            stats.usedBytes += area_size;
            stats.usedAreas++;
        } else {
            stats.freeBytes += area_size;
            stats.freeAreas++;
        }
        if(visitor != nullptr) visitor(&area, context);

        cursor += area_size;
    }

    return cursor == end_marker;
}

} // namespace

extern "C" bool heapVisitRegions(const HeapRegion_t *regions, HeapAreaVisitor_t visitor,
                                  void *context, HeapLayoutStats_t *stats)
{
    if(regions == nullptr || stats == nullptr) return false;

    *stats = {};
    for(const HeapRegion_t *region = regions; region->xSizeInBytes != 0U; ++region) {
        if(region->pucStartAddress == nullptr || !visit_region(*region, visitor, context, *stats))
            return false;
    }
    return true;
}

#ifndef HEAP_DEBUG_NO_TARGET_REGIONS

extern "C" uint8_t __MainHeapStart;
extern "C" uint8_t __MainHeapEnd;
extern "C" uint8_t __GeneralAHBStart;
extern "C" uint8_t __GeneralAHBEnd;

extern "C" bool heapVisitAreas(HeapAreaVisitor_t visitor, void *context, HeapLayoutStats_t *stats)
{
    const HeapRegion_t regions[] = {
        {&__MainHeapStart, static_cast<size_t>(&__MainHeapEnd - &__MainHeapStart)},
        {&__GeneralAHBStart, static_cast<size_t>(&__GeneralAHBEnd - &__GeneralAHBStart)},
        {nullptr, 0},
    };
    return heapVisitRegions(regions, visitor, context, stats);
}

#endif
