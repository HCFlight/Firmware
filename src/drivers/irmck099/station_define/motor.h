#ifndef _FC_STATION__INC__DEFINES__DYNAMIC__MOTOR_H_
#define _FC_STATION__INC__DEFINES__DYNAMIC__MOTOR_H_

#include "basic_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DYNAMIC_MOTOR_NUM   4

typedef int32_t motor_speed_t;

enum dynamic_motor_status_e {
    DYNAMIC_MOTOR_STATUS_STOP,     // 停转
    DYNAMIC_MOTOR_STATUS_STARTING, // 启动中
    DYNAMIC_MOTOR_STATUS_NORMAL,   // 正常运转
    DYNAMIC_MOTOR_STATUS_JAMMED,   // 卡住
    DYNAMIC_MOTOR_STATUS_NOLOAD,   // 空载
    DYNAMIC_MOTOR_STATUS_BROKEN    // 损坏
};

enum dynamic_motor_music_index_e {
    DYNAMIC_MOTOR_MUSIC_INDEX_POWER_ON,
    DYNAMIC_MOTOR_MUSIC_INDEX_POWER_OFF
};

#ifdef __cplusplus
}
#endif

#endif
