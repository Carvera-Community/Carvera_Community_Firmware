#ifndef CARVERA_HEAP_DEBUG_H
#define CARVERA_HEAP_DEBUG_H

#include "heap/heap_port.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HeapAreaInfo {
    const uint8_t *address;
    size_t size;
    bool allocated;
} HeapAreaInfo_t;

typedef struct HeapLayoutStats {
    size_t usedBytes;
    size_t freeBytes;
    size_t usedAreas;
    size_t freeAreas;
} HeapLayoutStats_t;

typedef void (*HeapAreaVisitor_t)(const HeapAreaInfo_t *area, void *context);

// The visitor runs while heap metadata is being traversed and must not allocate or free memory.
bool heapVisitRegions(const HeapRegion_t *regions, HeapAreaVisitor_t visitor,
                      void *context, HeapLayoutStats_t *stats);
bool heapVisitAreas(HeapAreaVisitor_t visitor, void *context, HeapLayoutStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
