#ifndef _FC_STATION__INC__DEFINES__F401__F401_H_
#define _FC_STATION__INC__DEFINES__F401__F401_H_

#include "gimbal.h"
#include "power_led.h"

//#include <station/basic_type.h>
#include "basic_type.h"

#ifdef __cplusplus
extern "C" {
#endif

struct f401_gimbal_raw_s {
    uint32_t cur_step;  // pitch 步数，向上为正，用 f401_gimbal_convert_step_to_angle 转化为 rad
    uint32_t max_step;  // pitch 步数，向上为正，用 f401_gimbal_convert_step_to_angle 转化为 rad
    uint16_t cur_speed; // pitch 角速度，方向在 direction 中，用f401_gimbal_convert_speed_to_ang_vel 转化为 rad/s
    uint8_t direction : 1;  // 向上为 1，向下为 0
    uint8_t uplimit_triggered : 1;   // 上限位开关状态，1 表示触发，0 表示未触发
    uint8_t downlimit_triggered : 1; // 下限位开关状态，1 表示触发，0 表示未触发
    uint8_t status : 4; // gimbal 状态
    uint8_t selfcheck_status : 4; // 自检状态
};

#ifdef __cplusplus
}
#endif

#endif
