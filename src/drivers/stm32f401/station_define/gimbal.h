#ifndef _FC_STATION__INC__DEFINES__F401__GIMBAL_H_
#define _FC_STATION__INC__DEFINES__F401__GIMBAL_H_

//#include <lib/math/math.h>

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI            3.14159265358979323846f // pi
#endif

#define F401_GIMBAL_RAD_PER_PULSE       ((M_PI * 18) / (180 * 47 * 32))
#define F401_GIMBAL_OFFSET_STEPS        4177.6498f
#define F401_GIMBAL_TIMER_CONVERT_PAR   (F401_GIMBAL_RAD_PER_PULSE * 1312500)

#define f401_gimbal_convert_step_to_angle(step) \
    (F401_GIMBAL_RAD_PER_PULSE * (step) - F401_GIMBAL_OFFSET_STEPS)
#define f401_gimbal_convert_angle_to_step(angle)    \
    (((angle) + F401_GIMBAL_OFFSET_STEPS) / F401_GIMBAL_RAD_PER_PULSE)

#define f401_gimbal_convert_speed_to_ang_vel(direction, speed) \
    (((speed) == 0xFFFF || (speed) == 0xFFFE) ? 0.0f : F401_GIMBAL_TIMER_CONVERT_PAR / ((speed) + 1)) * ((direction) ? 1 : -1)
#define f401_gimbal_convert_ang_vel_to_speed(ang_vel)   \
    ((ang_vel) == 0.0f ? 0xFFFF : F401_GIMBAL_TIMER_CONVERT_PAR / fabsf(ang_vel) - 1)

// F401 云台自检状态
enum f401_gimbal_selfcheck_status_e {
    F401_GIMBAL_SELFCHECK_STATUS_OK = 0,           // 正常
    F401_GIMBAL_SELFCHECK_STATUS_UP_LIMIT_ERROR,   // 上限位异常
    F401_GIMBAL_SELFCHECK_STATUS_DOWN_LIMIT_ERROR, // 下限位异常
    F401_GIMBAL_SELFCHECK_STATUS_BOTH_LIMIT_ERROR, // 两个限位都异常
    F401_GIMBAL_SELFCHECK_STATUS_MAX_NUM
};

// F401 云台状态
enum f401_gimbal_status_e {
    F401_GIMBAL_STATUS_INIT_TO_UP = 0,
    F401_GIMBAL_STATUS_INIT_UP_TO_DOWN,
    F401_GIMBAL_STATUS_INIT_WAIT_TRIG,
    F401_GIMBAL_STATUS_INIT_WAIT_TIM,
    F401_GIMBAL_STATUS_INIT_DOWN_TO_MID,
    F401_GIMBAL_STATUS_STOP,
    F401_GIMBAL_STATUS_RUN,
    F401_GIMBAL_STATUS_RUN_WAIT_TIM,
    F401_GIMBAL_STATUS_RELOCATE_TO_DOWN,
    F401_GIMBAL_STATUS_RELOCATE_WAIT_TRIG,
    F401_GIMBAL_STATUS_RELOCATE_TO_MID,
    F401_GIMBAL_STATUS_MAX_NUM
};

#ifdef __cplusplus
}
#endif

#endif
