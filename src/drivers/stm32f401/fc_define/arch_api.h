#ifndef _FC__INC__ARCH_API_H_
#define _FC__INC__ARCH_API_H_

#include "basic_type.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void arch_event_begin(uint8_t type, void *arg, int len);
extern void arch_event_end(uint8_t type);
extern void arch_event_imu_end(void);
extern void rc_check_packet(void);
extern void stats_timer_reg(void);

/**
 * Do the arch-specific initialization.
 *
 * Different architecture need different initialization code/tasks.
 * Every architecture should implement it's own arch_init function.
 *
 * @param argc  Num of arguments to run the flight-control program.
 * @param argv  Array of arguments to run the flight-control program.
 */
void arch_init(char **args);

/**
 * Init timer.
 */
int arch_absolute_time_init(void);

/**
 * Get current time to system startup in microseconds.
 * @return Current time to system startup in microseconds.
 */
uint64_t arch_absolute_time(void);

void arch_write_binlog(uint8_t *data, uint8_t length);

uint8_t arch_bool_compare_and_swap(volatile uint32_t *target, uint32_t compare, uint32_t swap);

#ifdef __cplusplus
}
#endif

#endif
