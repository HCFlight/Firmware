// motor.h
// Jin Pengxiang
// Created on 2017/2/6
// Copyright (c) 2015 Hangzhou Zero Zero Technology Co., Ltd. All rights reserved.

#ifndef _FC__DYNAMIC__MOTOR_H_
#define _FC__DYNAMIC__MOTOR_H_

#include "../station_define/motor.h"

#include "basic_type.h"

#ifdef __cplusplus
extern "C" {
#endif

struct dynamic_motor_raw_s {
    motor_speed_t speed[DYNAMIC_MOTOR_NUM];
};

struct dynamic_motor_output_s {
    motor_speed_t set_speed[DYNAMIC_MOTOR_NUM];
};

#if defined PROTO_HC

//#define DRIVER_MOTOR_SELECT_C8051     // PWM
#define DRIVER_MOTOR_SELECT_IRMCK099    // FOC

#if defined DRIVER_MOTOR_SELECT_C8051

#define DYNAMIC_MOTOR_SPEED_RATIO   11346372.0f
#define DYNAMIC_MOTOR_MAX_SET_SPEED (0xFF)
#define DYNAMIC_MOTOR_MIN_SET_SPEED 0

#elif defined DRIVER_MOTOR_SELECT_IRMCK099

#define DYNAMIC_MOTOR_RATE_CTRL_VALID

#define DYNAMIC_MOTOR_HOVER_SPEED   13800   // 标准悬停转速（桨叶完整时总升力 == 飞机自重）
#define DYNAMIC_MOTOR_MAX_SET_SPEED 18000
#define DYNAMIC_MOTOR_MIN_SET_SPEED 4500

#endif

#endif

#ifdef __cplusplus
}
#endif

#endif
