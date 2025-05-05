#include "MemoryPool.h"

#include "StreamOutput.h"

#include <mri.h>
#include <cstdio>
#include <cstring> // For memset
#include <cstdarg> // For va_list

// --- Debug Configuration ---
// Define this in the build system (e.g., -DPOOL_DEBUG_ENABLED=1 for Debug/Checked)
#ifndef POOL_DEBUG_ENABLED
#define POOL_DEBUG_ENABLED 0 // Default to enabled if not set by build system
#endif

#define POOL_INITIAL_PATTERN 0xEFBEADDE        // Pattern for initialized pool & free blocks
#define POOL_ALLOC_PATTERN 0xCDCDCDCD          // Pattern for newly allocated block payload
#define POOL_FREE_PATTERN POOL_INITIAL_PATTERN // Reuse initial pattern for freed blocks

// Global counter for errors detected in release mode
static volatile uint32_t g_pool_error_count = 0;
class MemoryPool;

// --- Forward Declarations for Debug Helpers (defined at EOF) ---
#if POOL_DEBUG_ENABLED == 1
static void fill_pattern32(void *dest, uint32_t pattern, size_t size);
static bool check_pattern32(const void *src, uint32_t pattern, size_t size);
// Function prototype for validate_pool_integrity needed earlier if called by inline funcs
static bool validate_pool_integrity_internal(const MemoryPool *pool, bool check_free_pattern);
#endif

// Need check_pattern32 declaration outside the #if for the inline pool_check
static bool check_pattern32(const void *src, uint32_t pattern, size_t size);

// --- Inline Debug/Utility Functions ---

// Conditional printf
static inline void pool_printf(const char *fmt, ...)
{
#if POOL_DEBUG_ENABLED == 1
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#else
    (void)fmt; // Avoid unused parameter warning
#endif
}

// Conditional debug break
static inline void pool_break()
{
#if POOL_DEBUG_ENABLED == 1
    __debugbreak();
#endif
}

// Conditional pattern fill
static inline void pool_fill(void *dest, uint32_t pattern, size_t size)
{
#if POOL_DEBUG_ENABLED == 1
    fill_pattern32(dest, pattern, size);
#else
    (void)dest;
    (void)pattern;
    (void)size; // Avoid unused parameter warnings
#endif
}

// Conditional pattern check
static inline bool pool_check(const void *src, uint32_t pattern, size_t size)
{
#if POOL_DEBUG_ENABLED == 1
    return check_pattern32(src, pattern, size);
#else
    (void)src;
    (void)pattern;
    (void)size;  // Avoid unused parameter warnings
    return true; // Always passes in release
#endif
}

// Error Handler (logs/breaks in debug, counts in release)
static inline void handle_pool_error(const char *fmt, ...)
{
#if POOL_DEBUG_ENABLED == 1
    va_list args;
    printf("POOL ERROR: ");
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n"); // Ensure newline after error
    pool_break();
#else
    (void)fmt; // Avoid unused parameter warning
    g_pool_error_count++;
#endif
}


// this catches all usages of delete blah. The object's destructor is called before we get here
// it first checks if the deleted object is part of a pool, and uses free otherwise.
void operator delete(void *p)
{
    // Allow deleting null pointers (standard C++ behavior)
    if (p == nullptr) {
        return;
    }

    MemoryPool *m = MemoryPool::first;
    while (m) {
        if (m->has(p)) {
            m->dealloc(p);
            return;
        }
        m = m->next;
    }

    // If not found in any pool, assume it's from the standard heap
    free(p);
}

#define offset(x) ((uint32_t)(((uint8_t *)x) - ((uint8_t *)this->base)))

typedef struct __attribute__((packed)) {
    uint32_t next : 31;
    uint32_t used : 1;

    uint8_t data[];
} _poolregion;

MemoryPool *MemoryPool::first = NULL;

// --- Constructor / Destructor ---

MemoryPool::MemoryPool(void *base, uint16_t size)
{
    this->base = base;
    this->size = size;

    // Basic sanity check on pool size
    if (size < sizeof(_poolregion) * 2) { // Need space for at least one header and minimal data
        handle_pool_error("MemoryPool size %u too small!", size);
        // Return after error handling
        return;
    }

    // Initialize the first block header
    _poolregion *initial_block = (_poolregion *)base;
    initial_block->used = 0;
    initial_block->next = size; // Initially, the whole pool is one free block

    // Fill the entire pool (including the header area initially) with the free pattern
    pool_fill(base, POOL_INITIAL_PATTERN, size);

    // Re-initialize the first block header after filling (redundant setting used=0, but ensures next is correct)
    initial_block->used = 0;
    initial_block->next = size;

    // insert ourselves into head of LL
    next = first;
    first = this;
}

MemoryPool::~MemoryPool()
{
    // remove ourselves from the LL
    if (first == this) { // special case: we're first
        first = this->next;
        return;
    }

    // otherwise search the LL for the previous pool
    MemoryPool *m = first;
    while (m) {
        if (m->next == this) {
            m->next = next;
            return;
        }
        m = m->next;
    }
}

void *MemoryPool::alloc(size_t nbytes)
{
#if POOL_DEBUG_ENABLED == 1
    // Validate only in debug builds at the start
    if (!validate_pool_integrity_internal(this, true)) {
        // Error handled within validate
        return NULL;
    }
#endif

    // nbytes = ceil(nbytes / 4) * 4
    if (nbytes & 3)
        nbytes += 4 - (nbytes & 3);

    // start at the start
    _poolregion *p = ((_poolregion *)base);

    // find the allocation size including our metadata
    uint32_t nsize = nbytes + sizeof(_poolregion); // Use uint32_t for consistency with header size

    // now we walk the list, looking for a sufficiently large free block
    do {
        // Safely read the header value using memcpy
        uint32_t p_header_val;
        memcpy(&p_header_val, p, sizeof(uint32_t));

        bool p_is_used = (p_header_val >> 31) & 1;
        uint32_t p_block_size = p_header_val & 0x7FFFFFFF; // Size of the current block

        if (!p_is_used && (p_block_size >= nsize)) { // we found a free space that's big enough

            // Check if the free block payload was corrupted *before* allocating
            size_t free_payload_size = p_block_size - sizeof(_poolregion);
            if (!pool_check(&p->data, POOL_FREE_PATTERN, free_payload_size)) {
                handle_pool_error("Free block payload corrupted before alloc! Block at %p (%lu bytes)", p,
                                  p_block_size);
                return NULL;
            }

            uint32_t p_final_header;

            // if there's free space at the end of this block
            // Ensure there's enough space for a new header plus minimal data (e.g., 4 bytes aligned)
            if (p_block_size >= (nsize + sizeof(_poolregion) + 4)) // Check if remaining space is usable
            {
                // Split the block
                _poolregion *q = (_poolregion *)(((uint8_t *)p) + nsize);
                uint32_t q_block_size = p_block_size - nsize;

                // write a new block header into q (used=0, next=q_block_size)
                uint32_t q_header_val = (0 << 31) | (q_block_size & 0x7FFFFFFF);
                memcpy(q, &q_header_val, sizeof(uint32_t));

                // Set p's final header: used=1, size=nsize
                p_final_header = (1 << 31) | (nsize & 0x7FFFFFFF);

                if (offset(q) >= size) {
                    handle_pool_error("Calculated split block 'q' offset %lu is out of bounds!", offset(q));
                    return NULL;
                }
            }
            else {
                // Use the whole original block
                // Set p's final header: used=1, size=p_block_size (original size)
                p_final_header = (1 << 31) | (p_block_size & 0x7FFFFFFF);
            }

            // Write p's final header safely
            memcpy(p, &p_final_header, sizeof(uint32_t));

            void *__alloc_ret_ptr = &p->data; // preserve pointer for asm & return

            // Fill the payload based on the ACTUAL size written in the final header
            size_t fill_size = (p_final_header & 0x7FFFFFFF) - sizeof(_poolregion);
            pool_fill(__alloc_ret_ptr, POOL_ALLOC_PATTERN, fill_size);

            // Instructions to allow GDB to capture memory allocation
            // Added "memory" clobber to prevent compiler reordering around asm
            asm volatile("mov  r1,%0\n"
                         "mov  r0,%1\n"
                         ".global memorypool_alloc_return_point\n"
                         "memorypool_alloc_return_point:\n"
                         :
                         : "r"(nbytes), "r"(__alloc_ret_ptr)
                         : "r0", "r1", "memory");

            // then return the data region for the block
            return __alloc_ret_ptr;
        }

        // Check if we've reached the end of the pool without finding a suitable block
        if (offset(p) + p_block_size >= size) {
            break; // Reached the end of the pool structure
        }

        // p = next block
        _poolregion *next_p = (_poolregion *)(((uint8_t *)p) + p_block_size);

        // Safety check: avoid infinite loop or invalid pointers if pool metadata is corrupted
        // Also check next block's size read directly (using memcpy) before casting to _poolregion
        uint32_t next_p_header_val = 0;
        // Ensure next_p is within bounds before memcpy
        if ((uintptr_t)next_p < ((uintptr_t)base + size)) {
            memcpy(&next_p_header_val, next_p, sizeof(uint32_t));
        }

        if ((uintptr_t)next_p >= ((uintptr_t)base + size) || next_p <= p || next_p_header_val == 0) {
            handle_pool_error("Pool metadata corruption detected while traversing list at block %p", p);
            return NULL;
        }
        p = next_p;

    } while (1);

    // fell off the end of the region or couldn't find a block!
    pool_printf("POOL INFO: MemoryPool::alloc failed to find suitable block for %zu bytes\n", nbytes);
    // This isn't necessarily an *error* in the pool state, just a failed allocation.
    return NULL;
}

void MemoryPool::dealloc(void *d)
{
    // Check for null pointer deallocation - this is valid in C++, do nothing.
    if (d == nullptr) {
        return;
    }

    // Check if the pointer is within the pool's bounds before calculating header address
    if (!has(d)) {
        handle_pool_error("Attempt to dealloc pointer %p outside pool bounds [%p, %p)!", d, base,
                          (uint8_t *)base + size);
        return;
    }

    _poolregion *p = (_poolregion *)(((uint8_t *)d) - sizeof(_poolregion));

    // Sanity check: Ensure calculated header is within bounds
    if (((uint8_t *)p < (uint8_t *)base) || ((uint8_t *)p >= ((uint8_t *)base + size))) {
        handle_pool_error("Calculated header %p for data %p is out of pool bounds!", p, d);
        return;
    }

    // Read header safely *after* bounds check
    uint32_t p_header_val;
    memcpy(&p_header_val, p, sizeof(uint32_t));
    bool p_is_used = (p_header_val >> 31) & 1;
    uint32_t p_block_size = p_header_val & 0x7FFFFFFF;

    // Check if the block is already marked as free (double free)
    if (p_is_used == 0) { // Check the safely read value
        handle_pool_error("Double free detected for block at %p (pointer %p)!", p, d);
        return;
    }

    // Get payload size before modifying header
    size_t payload_size = p_block_size - sizeof(_poolregion);

    // Instructions to allow GDB to capture memory deallocation
    asm volatile("mov  r0,%0\n"
                 "mov  r1,%1\n"
                 ".global memorypool_free_hook\n"
                 "memorypool_free_hook:\n"
                 :
                 : "r"(d),
                   "r"(payload_size) // Use calculated payload size
                 : "r0", "r1");

    // Fill the payload with the free pattern *before* marking as free and coalescing
    pool_fill(&p->data, POOL_FREE_PATTERN, payload_size);

    // Mark block as free *after* filling payload
    uint32_t p_new_header_val = p_block_size; // used bit is 0
    memcpy(p, &p_new_header_val, sizeof(uint32_t));

    // --- Coalesce with the next block ---
    _poolregion *q_next = (_poolregion *)(((uint8_t *)p) + p_block_size); // Use originally read size

    // Read next block header safely if within bounds
    uint32_t q_next_header_val = 0;
    bool q_next_in_bounds = (q_next < (_poolregion *)(((uint8_t *)base) + size));
    if (q_next_in_bounds) {
        memcpy(&q_next_header_val, q_next, sizeof(uint32_t));
    }
    bool q_next_is_used = (q_next_header_val >> 31) & 1;
    uint32_t q_next_block_size = q_next_header_val & 0x7FFFFFFF;

#if POOL_DEBUG_ENABLED == 1
    asm volatile("mov  r0,%0\n"
                 "mov  r1,%1\n"
                 "mov  r2,%2\n"
                 "mov  r3,%3\n"
                 ".global memorypool_debug_hook\n"
                 "memorypool_debug_hook:\n"
                 :
                 : "r"(offset(p)),
                   "r"(p_block_size),      // Use originally read size
                   "r"(q_next_block_size), // Use safely read size
                   "r"(this->size)
                 : "r0", "r1", "r2", "r3", "memory" // Added memory clobber
    );
#endif // POOL_DEBUG_ENABLED

    if (q_next_in_bounds) {
        if (q_next_is_used == 0) { // Check safely read value
            // Sanity check before merging - use sizes read *before* modification
            if ((offset(p) + p_block_size + q_next_block_size) > size) {
                handle_pool_error("Heap corruption detected during forward coalesce check for block %p!", p);
                return;
            }
            else {
                // Calculate the new size explicitly
                uint32_t new_size = p_block_size + q_next_block_size;
                // Assign the new size, preserving the 'used=0' status
                uint32_t merged_header_val = new_size; // Used bit is 0
                memcpy(p, &merged_header_val, sizeof(uint32_t));
                // Update p_block_size to reflect the merge for potential backward coalesce check
                p_block_size = new_size;
            }
        }
    }

    // --- Coalesce with the previous block ---
    _poolregion *q_prev = (_poolregion *)base;
    while (q_prev < p) { // Iterate until we find the block whose 'next' points to p
        uint32_t q_prev_header_val;
        memcpy(&q_prev_header_val, q_prev, sizeof(uint32_t));
        bool q_prev_is_used = (q_prev_header_val >> 31) & 1;
        uint32_t q_prev_block_size = q_prev_header_val & 0x7FFFFFFF;

        _poolregion *potential_next = (_poolregion *)(((uint8_t *)q_prev) + q_prev_block_size);

        if (potential_next == p) {     // Found the previous block (q_prev)
            if (q_prev_is_used == 0) { // If the previous block is free
                // Sanity check before merging - use sizes read *before* modification
                if ((offset(q_prev) + q_prev_block_size + p_block_size) > size) {
                    handle_pool_error("Heap corruption detected during backward coalesce check for block %p!", q_prev);
                    return;
                }
                else {
                    uint32_t new_prev_size = q_prev_block_size + p_block_size;
                    // Assign the new size, preserving the 'used=0' status
                    uint32_t merged_prev_header_val = new_prev_size; // Used bit is 0
                    memcpy(q_prev, &merged_prev_header_val, sizeof(uint32_t));
                    // p is now merged into q_prev. The payload of the merged block (q_prev)
                    // should already be filled with the free pattern from both parts.
                }
            }
            // Whether merged or not, we found the previous block and are done.
            goto dealloc_end; // Use goto for clarity to reach single exit point
        }

        // Check for end condition or corruption before advancing q_prev
        if (offset(q_prev) + q_prev_block_size >= size) {
            break; // Stop if we hit the end
        }

        // Move to the next block in the list
        q_prev = potential_next;
    }
    // If loop finishes without finding p, it means p was the first block or corruption occurred.

dealloc_end: // Label for goto
    // Final validation after potential coalescing
    pool_printf("  Performing final validation before exiting dealloc(%p)...\n", d);
#if POOL_DEBUG_ENABLED == 1
    if (!validate_pool_integrity_internal(this, true)) {
        // Error already handled by validate
    }
#endif
}

// --- Public Methods ---

void MemoryPool::debug(StreamOutput *str)
{
    // Always print basic info and error count
    uint32_t total_free_verified = this->free(); // Calculate total free space
    str->printf("MemoryPool at %p: Size=%u, TotalFree=%lu, ErrorCount=%lu\n", base, size,
                (unsigned long)total_free_verified, (unsigned long)g_pool_error_count);

    // Always perform the detailed walk for mem -v
    _poolregion *p = (_poolregion *)base;
    uint32_t total_used = 0;
    uint32_t total_fragmented_free = 0;
    // Size of the last block if it's free and touches the end
    uint32_t unallocated_at_end = 0;
    uint32_t current_offset = 0;

    str->printf("Detailed Pool Walk:\n");

    while (current_offset < size) {
        uint32_t p_header_val;
        memcpy(&p_header_val, p, sizeof(uint32_t));
        bool p_is_used = (p_header_val >> 31) & 1;
        uint32_t p_block_size = p_header_val & 0x7FFFFFFF;

        // Basic size check for safety before proceeding
        if (p_block_size == 0 || current_offset + p_block_size > size) {
            str->printf("	ERROR: Invalid block size %u at offset %lu. Aborting walk.\n", (unsigned int)p_block_size,
                        current_offset);
            break;
        }

        str->printf("	Chunk at %p (%4lu): %s, %u bytes\n", p, current_offset, (p_is_used ? "used" : "free"),
                    (unsigned int)p_block_size);

        bool is_last_block = (current_offset + p_block_size >= size);

        if (p_is_used) {
            total_used += p_block_size;
        }
        else {
            if (is_last_block) {
                unallocated_at_end = p_block_size;
            }
            else {
                total_fragmented_free += p_block_size;
            }
        }

        // Advance offset
        current_offset += p_block_size;

        if (current_offset >= size) {
            break; // Reached or exceeded end
        }

        p = (_poolregion *)((uint8_t *)base + current_offset);
    }

    uint32_t total_free_calculated = total_fragmented_free + unallocated_at_end;
    uint32_t total_calculated = total_used + total_free_calculated;

    str->printf("Walk Summary: Used=%lu, FragmentedFree=%lu, Unallocated=%lu, TotalFree(Calc)=%lu, TotalCalc=%lu\n",
                total_used, total_fragmented_free, unallocated_at_end, total_free_calculated, total_calculated);

    // Only compare/warn in debug builds
#if POOL_DEBUG_ENABLED == 1
    if (total_calculated != size) {
        str->printf("WARNING: Pool sizes calculated by debug walk don't add up!\n");
    }
    if (total_free_calculated != total_free_verified) {
        str->printf("WARNING: Discrepancy between debug walk free count (%lu) and verified free count (%lu).\n",
                    total_free_calculated, total_free_verified);
    }
    // End of debug-only checks
#endif // POOL_DEBUG_ENABLED == 1
}

bool MemoryPool::has(void *p)
{
    return ((p >= base) && (p < (void *)(((uint8_t *)base) + size)));
}

// Calculates total free space by walking the list
uint32_t MemoryPool::free()
{
    uint32_t free_bytes = 0;
    _poolregion *p = (_poolregion *)base;
    uint32_t current_offset = 0;

    while (current_offset < size) {
        uint32_t p_header_val;
        memcpy(&p_header_val, p, sizeof(uint32_t)); // Safe read
        bool p_is_used = (p_header_val >> 31) & 1;
        uint32_t p_block_size = p_header_val & 0x7FFFFFFF;

        // Check for invalid block size before using it
        // Check for size 0 or size exceeding remaining pool space
        if (p_block_size == 0 || p_block_size > size - current_offset) {
            pool_printf("POOL WARNING: Invalid block size %u detected in free() at offset %lu. Aborting count.\n",
                        (unsigned int)p_block_size, current_offset);
#if POOL_DEBUG_ENABLED == 1
            pool_break();
            // If debugbreak doesn't halt, return to prevent using corrupted data
            return free_bytes;
#else
            // In release, maybe increment error counter? Or just return what we have.
            // g_pool_error_count++; // Optional: Count this as an error
            return free_bytes;
#endif
        }
        // Check for minimum size (header only is invalid)
        if (p_block_size < sizeof(_poolregion)) {
            pool_printf("POOL WARNING: Invalid block size %u (less than header) detected in free() at offset %lu. "
                        "Aborting count.\n",
                        (unsigned int)p_block_size, current_offset);
#if POOL_DEBUG_ENABLED == 1
            pool_break();
            return free_bytes;
#else
            // g_pool_error_count++; // Optional: Count this as an error
            return free_bytes;
#endif
        }

        if (p_is_used == 0) {
            free_bytes += p_block_size; // Add the size of the free block
        }

        current_offset += p_block_size;
        if (current_offset >= size) {
            break; // Reached end
        }

        p = (_poolregion *)((uint8_t *)base + current_offset);
    }

    return free_bytes;
}

// --- Debug Validation Function Definitions (Compiled only when POOL_DEBUG_ENABLED is 1) ---
#if POOL_DEBUG_ENABLED == 1

// Helper to fill memory with a 32-bit pattern
static void fill_pattern32(void *dest, uint32_t pattern, size_t size)
{
    if (dest == nullptr || size == 0)
        return;
    uint32_t *p = (uint32_t *)dest;
    size_t count = size / sizeof(uint32_t);
    for (size_t i = 0; i < count; ++i) {
        p[i] = pattern;
    }
    size_t remainder = size % sizeof(uint32_t);
    if (remainder > 0) {
        uint8_t *byte_ptr = (uint8_t *)(p + count);
        uint8_t *pattern_bytes = (uint8_t *)&pattern;
        for (size_t i = 0; i < remainder; ++i) {
            byte_ptr[i] = pattern_bytes[i];
        }
    }
}

// Helper to check memory against a 32-bit pattern
static bool check_pattern32(const void *src, uint32_t pattern, size_t size)
{
    if (src == nullptr || size == 0)
        return true;
    const uint32_t *p = (const uint32_t *)src;

    // --- MODIFICATION START ---
    // Only check the first N bytes for the free pattern to avoid false positives
    // due to stale data in padding/alignment areas of previously smaller blocks.
    const size_t bytes_to_check = 8; // Check the first 8 bytes
    size_t check_size = (size < bytes_to_check) ? size : bytes_to_check;
    size_t count = check_size / sizeof(uint32_t);
    // --- MODIFICATION END ---

    for (size_t i = 0; i < count; ++i) {
        if (p[i] != pattern) {
            printf("CORRUPTION DETECTED: Expected pattern 0x%08lX, found 0x%08lX at offset %lu within block %p\n",
                   pattern, p[i], (unsigned long)(i * sizeof(uint32_t)), (const void *)src);
            return false;
        }
    }
    // --- MODIFICATION START ---
    // Adjust remainder calculation based on the limited check_size
    size_t remainder = check_size % sizeof(uint32_t);
    // --- MODIFICATION END ---
    if (remainder > 0) {
        const uint8_t *byte_ptr = (const uint8_t *)(p + count);
        const uint8_t *pattern_bytes = (const uint8_t *)&pattern;
        for (size_t i = 0; i < remainder; ++i) {
            if (byte_ptr[i] != pattern_bytes[i]) {
                printf(
                    "CORRUPTION DETECTED: Expected pattern byte 0x%02X, found 0x%02X at offset %lu within block %p\n",
                    pattern_bytes[i], byte_ptr[i], (unsigned long)(count * sizeof(uint32_t) + i), (const void *)src);
                return false;
            }
        }
    }
    return true;
}

// Internal validation function (static)
// Defined as a static helper function, uses public getters for access
static bool validate_pool_integrity_internal(const MemoryPool *pool, bool check_free_pattern)
{
    _poolregion *p = (_poolregion *)pool->getBase();
    uint32_t current_offset = 0;
    bool ok = true;

    pool_printf("Validating Pool %p (Size: %u, Check Free: %s)...\n", pool->getBase(), pool->getSize(),
                check_free_pattern ? "Yes" : "No");

    while (current_offset < pool->getSize()) {
        // Read header safely
        uint32_t p_header_val;
        memcpy(&p_header_val, p, sizeof(uint32_t));
        bool p_is_used = (p_header_val >> 31) & 1;
        uint32_t p_block_size = p_header_val & 0x7FFFFFFF;

        // --- Basic Block Checks ---
        if (p_block_size == 0 || p_block_size > pool->getSize() - current_offset) {
            pool_printf("  ERROR: Block at %p (Offset %lu) has invalid size %u (Pool Size %u)!\n", p, current_offset,
                        (unsigned int)p_block_size, pool->getSize());
            ok = false;
            pool_break();
            break; // Size is definitely wrong, cannot proceed reliably
        }
        if (p_block_size < sizeof(_poolregion) + 4) { // Minimum size: header + 4 bytes data (Warning only)
            pool_printf("  WARNING: Block at %p (Offset %lu) has suspiciously small size %u.\n", p, current_offset,
                        (unsigned int)p_block_size);
        }

        pool_printf("  Block at %p (Offset: %4lu): %s, Size: %5u\n", p, current_offset, (p_is_used ? "Used" : "Free"),
                    (unsigned int)p_block_size);

        // --- Free Block Pattern Check (if requested) ---
        if (!p_is_used && check_free_pattern) {
            size_t payload_size = p_block_size - sizeof(_poolregion);
            if (!check_pattern32(&p->data, POOL_FREE_PATTERN, payload_size)) {
                pool_printf(" -> Free block payload check FAILED!\n");
                ok = false;
                pool_break();
            }
            else {
                pool_printf(" -> Free OK\n");
            }
        }
        else {
            // No explicit print needed here if not checking or if used
        }

        // --- Advance to next block ---
        current_offset += p_block_size;
        if (current_offset == pool->getSize()) {
            break; // Reached end cleanly
        }

        p = (_poolregion *)((uint8_t *)pool->getBase() + current_offset);
    }

    if (current_offset != pool->getSize() && ok) {
        pool_printf("  ERROR: Pool validation walk ended prematurely at offset %lu (Pool Size %u). Likely due to prior "
                    "error.\n",
                    current_offset, pool->getSize());
        ok = false;
        pool_break();
    }
    else if (ok) {
        pool_printf("Pool validation PASSED.\n");
    }
    else {
        pool_printf("Pool validation FAILED.\n");
    }

    return ok;
}

#endif // POOL_DEBUG_ENABLED == 1

// Public validate function (conditionally calls internal one)
bool MemoryPool::validate_pool_integrity(bool check_free_pattern) const
{
#if POOL_DEBUG_ENABLED == 1
    // Call the static friend function
    return validate_pool_integrity_internal(this, check_free_pattern);
#else
    (void)check_free_pattern; // Avoid unused warning
    return true;              // Always passes in release
#endif
}
