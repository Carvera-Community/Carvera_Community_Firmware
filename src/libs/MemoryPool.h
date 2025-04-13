#ifndef _MEMORYPOOL_H
#define _MEMORYPOOL_H

#include <cstdint>
// #include <cstdio>
#include <cstdlib>

class StreamOutput;

/*
 * with MUCH thanks to http://www.parashift.com/c++-faq-lite/memory-pools.html
 *
 * test framework at https://gist.github.com/triffid/5563987
 */

class MemoryPool
{
public:
    MemoryPool(void* base, uint16_t size);
    ~MemoryPool();

    void* alloc(size_t);
    void  dealloc(void* p);

    void  debug(StreamOutput*);

    bool  has(void*);

    uint32_t free(void);

    // Getters for internal state (used by debug validation)
    void* getBase() const { return base; }
    uint16_t getSize() const { return size; }

    // Validate the integrity of the pool structure and optionally check free block patterns
    bool validate_pool_integrity(bool check_free_pattern) const;

    MemoryPool* next;

    static MemoryPool* first;

private:
    void* base;
    uint16_t size;

    #if POOL_DEBUG_ENABLED == 1
    // Friend function for validation (defined in cpp)
    friend bool validate_pool_integrity_internal(const MemoryPool* pool, bool check_free_pattern);
    #endif
};

// this overloads "placement new"
inline void* operator new(size_t nbytes, MemoryPool& pool)
{
    return pool.alloc(nbytes);
}

// this allows placement new to free memory if the constructor fails
inline void  operator delete(void* p, MemoryPool& pool)
{
    pool.dealloc(p);
}

extern MemoryPool AHB; // the main AHB memory pool

#endif /* _MEMORYPOOL_H */
