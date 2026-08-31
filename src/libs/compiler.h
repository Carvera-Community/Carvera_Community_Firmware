#ifndef COMPILER_H
#define COMPILER_H

#if defined(__GNUC__) || defined(__clang__)
#define LOCATED_IN_AHBSRAM __attribute__((section("AHBSRAM")))
#define ALIGNED_TO(bytes) __attribute__((aligned(bytes)))
#else
#define LOCATED_IN_AHBSRAM
#define ALIGNED_TO(bytes)
#endif

#endif
