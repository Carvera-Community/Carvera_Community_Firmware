#ifndef _PLATFORM_MEMORY_H
#define _PLATFORM_MEMORY_H

#ifndef AHB
    #include "MemoryPool.h"

    #define AHB (*_ahb)

    extern MemoryPool* _ahb;
#endif

#endif /* _PLATFORM_MEMORY_H */
