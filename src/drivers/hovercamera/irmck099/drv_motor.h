#ifndef _HEXAGON__DRIVER__DRV_MOTOR_H_
#define _HEXAGON__DRIVER__DRV_MOTOR_H_

#include "fc_define/motor.h"

#define DRIVER_MOTOR_SELECT_IRMCK099 1

#if defined DRIVER_MOTOR_SELECT_C8051   // PWM

#define DRV_MOTOR_DRIVER_CYCLE_FREQ_HZ  1000

#elif defined DRIVER_MOTOR_SELECT_IRMCK099  // FOC

#define DRV_MOTOR_DRIVER_CYCLE_FREQ_HZ  500

#else

#define DRV_MOTOR_DRIVER_CYCLE_FREQ_HZ  1000

#endif

int drv_motor_init(void);
int drv_motor_update(const struct dynamic_motor_output_s *motor_output);
int drv_motor_get_data(struct dynamic_motor_raw_s *motor_data);

#endif
