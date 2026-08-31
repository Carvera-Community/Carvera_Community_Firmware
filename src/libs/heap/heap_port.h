#ifndef CARVERA_HEAP_PORT_H
#define CARVERA_HEAP_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t BaseType_t;
typedef uintptr_t portPOINTER_SIZE_TYPE;

typedef struct HeapRegion {
    uint8_t *pucStartAddress;
    size_t xSizeInBytes;
} HeapRegion_t;

typedef struct HeapStats {
    size_t xAvailableHeapSpaceInBytes;
    size_t xSizeOfLargestFreeBlockInBytes;
    size_t xSizeOfSmallestFreeBlockInBytes;
    size_t xNumberOfFreeBlocks;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xNumberOfSuccessfulAllocations;
    size_t xNumberOfSuccessfulFrees;
} HeapStats_t;

void heapPortAssertFailed(const char *expression, const char *file, int line);

#ifdef __cplusplus
}
#endif

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configHEAP_CLEAR_MEMORY_ON_FREE 0
#define configENABLE_HEAP_PROTECTOR 0
#define configUSE_MALLOC_FAILED_HOOK 0

#define portBYTE_ALIGNMENT 8U
#define portBYTE_ALIGNMENT_MASK (portBYTE_ALIGNMENT - 1U)

#define PRIVILEGED_FUNCTION
#define PRIVILEGED_DATA
#define mtCOVERAGE_TEST_MARKER() ((void)0)
#define traceMALLOC(pointer, size) ((void)(pointer), (void)(size))
#define traceFREE(pointer, size) ((void)(pointer), (void)(size))
#define vTaskSuspendAll() ((void)0)
#define xTaskResumeAll() ((BaseType_t)0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define configASSERT(condition) \
    do { \
        if (!(condition)) heapPortAssertFailed(#condition, __FILE__, __LINE__); \
    } while (0)

#endif
