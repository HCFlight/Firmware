//#include <fc/arch_api.h>
#include "fc_define/arch_api.h"

static inline uint32_t
qurt_atomic_compare_val_and_set(volatile uint32_t *target,
    uint32_t old_val,
    uint32_t new_val)
{
    unsigned int current_val;

    __asm__ __volatile__(
        "1:     %0 = memw_locked(%2)\n"
        "       p0 = cmp.eq(%0, %3)\n"
        "       if !p0 jump 2f\n"
        "       memw_locked(%2, p0) = %4\n"
        "       if !p0 jump 1b\n"
        "2:\n"
        : "=&r" (current_val), "+m" (*target)
        : "r" (target), "r" (old_val), "r" (new_val)
        : "p0");

    return current_val;
}


inline uint8_t arch_bool_compare_and_swap(volatile uint32_t *target, uint32_t compare, uint32_t swap)
{
    return compare == qurt_atomic_compare_val_and_set(target, compare, swap);
}
