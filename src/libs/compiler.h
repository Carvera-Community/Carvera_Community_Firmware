#ifndef COMPILER_H
#define COMPILER_H

#if defined(__GNUC__) || defined(__clang__)
#define LOCATED_IN_AHBSRAM __attribute__((section("AHBSRAM")))
#define ALIGNED_TO(bytes) __attribute__((aligned(bytes)))
#else
#define LOCATED_IN_AHBSRAM
#define ALIGNED_TO(bytes)
#endif

#include <stdint.h>

extern unsigned int __StackLimit;
#define CONFIG_CACHE_STORAGE(Type, Capacity) \
    reinterpret_cast<Type *>((uintptr_t)&__StackLimit - (Capacity) * sizeof(Type))

#endif
