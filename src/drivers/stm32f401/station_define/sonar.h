#ifndef _FC_STATION__INC__DEFINES__SENSOR__SONAR_H_
#define _FC_STATION__INC__DEFINES__SENSOR__SONAR_H_

#include "basic_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PEAK_NUM    5
struct sensor_sonar_raw_s {
    uint16_t pulse_num;
    uint16_t noise;
    uint16_t falling_mm;
    uint16_t dead_zone_mm;
    uint16_t distance_mm[MAX_PEAK_NUM];
    uint16_t peak_value[MAX_PEAK_NUM];
    uint32_t framenum;
};

enum sensor_sonar_status_e {
    SENSOR_SONAR_STATUS_NOT_UPDATED,     // 超时未更新
    SENSOR_SONAR_STATUS_AVAILABLE,       // 可用
    SENSOR_SONAR_STATUS_LOW_RELIABILITY, // 可信度低，与运动状态不符
    SENSOR_SONAR_STATUS_WEAK_ECHO,       // 回波较弱，多为草地、沙地等疏松地面
    SENSOR_SONAR_STATUS_DEAD_ZONE,       // 死区
    SENSOR_SONAR_STATUS_COVERED,         // 被覆盖
    SENSOR_SONAR_STATUS_OUT_OF_RANGE,    // 超出距离
    SENSOR_SONAR_STATUS_NO_SIGNAL,       // 无信号（一般为传感器失效）
    SENSOR_SONAR_STATUS_STANDBY
};

#ifdef __cplusplus
}
#endif

#endif
