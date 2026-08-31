#include "heap/heap_api.h"
#include "heap/heap_5.h"

#include <cmsis.h>
#include <mri.h>
#include <cstdlib>
#include <cerrno>
#include <new>
#include <reent.h>

extern "C" unsigned char __MainHeapStart;
extern "C" unsigned char __MainHeapEnd;
extern "C" unsigned char __GeneralAHBStart;
extern "C" unsigned char __GeneralAHBEnd;

extern "C" void heapInitialize(void)
{
    const HeapRegion_t regions[] = {
        {&__MainHeapStart, static_cast<size_t>(&__MainHeapEnd - &__MainHeapStart)},
        {&__GeneralAHBStart, static_cast<size_t>(&__GeneralAHBEnd - &__GeneralAHBStart)},
        {nullptr, 0},
    };
    vPortDefineHeapRegions(regions);
}

extern "C" void heapPortAssertFailed(const char *, const char *, int)
{
    __debugbreak();
    abort();
}

static void reject_interrupt_allocation()
{
    if (__get_IPSR() != 0) heapPortAssertFailed("heap operation in interrupt", __FILE__, __LINE__);
}

static void *allocate(size_t size)
{
    reject_interrupt_allocation();
    return pvPortMalloc(size);
}

static void *allocate_zeroed(size_t count, size_t size)
{
    reject_interrupt_allocation();
    return pvPortCalloc(count, size);
}

extern "C" void *__wrap_malloc(size_t size)
{
    void *pointer = allocate(size);
    if(pointer == nullptr && size != 0) errno = ENOMEM;
    return pointer;
}

extern "C" void *__wrap_calloc(size_t count, size_t size)
{
    void *pointer = allocate_zeroed(count, size);
    if(pointer == nullptr && count != 0 && size != 0) errno = ENOMEM;
    return pointer;
}

extern "C" void __wrap_free(void *pointer)
{
    reject_interrupt_allocation();
    vPortFree(pointer);
}

extern "C" void *__wrap__malloc_r(_reent *context, size_t size)
{
    void *pointer = allocate(size);
    if(pointer == nullptr && size != 0) context->_errno = ENOMEM;
    return pointer;
}

extern "C" void *__wrap__calloc_r(_reent *context, size_t count, size_t size)
{
    void *pointer = allocate_zeroed(count, size);
    if(pointer == nullptr && count != 0 && size != 0) context->_errno = ENOMEM;
    return pointer;
}

extern "C" void __wrap__free_r(_reent *, void *pointer)
{
    __wrap_free(pointer);
}

extern "C" void *__wrap__realloc_r(_reent *, void *, size_t)
{
    heapPortAssertFailed("realloc is not supported", __FILE__, __LINE__);
    return nullptr;
}

void *operator new(size_t size)
{
    void *pointer = __wrap_malloc(size);
    if (pointer == nullptr) abort();
    return pointer;
}

void *operator new[](size_t size)
{
    return operator new(size);
}

void *operator new(size_t size, const std::nothrow_t &) noexcept
{
    return __wrap_malloc(size);
}

void *operator new[](size_t size, const std::nothrow_t &) noexcept
{
    return __wrap_malloc(size);
}

void operator delete(void *pointer) noexcept { __wrap_free(pointer); }
void operator delete[](void *pointer) noexcept { __wrap_free(pointer); }
void operator delete(void *pointer, size_t) noexcept { __wrap_free(pointer); }
void operator delete[](void *pointer, size_t) noexcept { __wrap_free(pointer); }
void operator delete(void *pointer, const std::nothrow_t &) noexcept { __wrap_free(pointer); }
void operator delete[](void *pointer, const std::nothrow_t &) noexcept { __wrap_free(pointer); }
