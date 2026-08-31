#ifndef CARVERA_HEAP_5_H
#define CARVERA_HEAP_5_H

#include "heap/heap_port.h"

#ifdef __cplusplus
extern "C" {
#endif

void vPortDefineHeapRegions(const HeapRegion_t *regions);
void *pvPortMalloc(size_t bytes);
void vPortFree(void *pointer);
void *pvPortCalloc(size_t count, size_t bytes);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
void xPortResetHeapMinimumEverFreeHeapSize(void);
void vPortGetHeapStats(HeapStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
