#ifndef _FC__SENSOR__SONAR_H_
#define _FC__SENSOR__SONAR_H_

#include <math.h>
//#include <station/defines/sensor/sonar.h>
#include "../station_define/sonar.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_SONAR_DELAY_MS       50 // ms

#define SENSOR_SONAR_RAW_FREQ_HZ    20

#define SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MIN_MM    400
#define SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MAX_MM    5100

#define SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MIN       ((float)(SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MIN_MM + 100) / 1000) // m
#define SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MAX       ((float)(SENSOR_SONAR_BELIEVABLE_ECHO_DISTANCE_MAX_MM - 100) / 1000) // m

#define SENSOR_SONAR_MAX_ANG        (45.0f / 180 * M_PI)
#define SENSOR_SONAR_MAX_ANG_COS    0.70710678118654752440084436210485f //(cosf(SENSOR_SONAR_MAX_ANG))

// 声呐数据状态
enum sensor_sonar_data_status_e {
    SENSOR_SONAR_DATA_STATUS_AVAILABLE,         // 可用
    SENSOR_SONAR_DATA_STATUS_LOW_RELIABILITY,   // 可信度低，与运动状态不符
    SENSOR_SONAR_DATA_STATUS_WEAK_ECHO,         // 回波较弱，多为草地、沙地等疏松地面
    SENSOR_SONAR_DATA_STATUS_DEAD_ZONE,         // 死区
    SENSOR_SONAR_DATA_STATUS_NO_ECHO,           // 无回波，一般为被覆盖（cover）或超出距离（out of range）
    SENSOR_SONAR_DATA_STATUS_NO_SIGNAL,         // 无信号，一般为传感器失效
    SENSOR_SONAR_DATA_STATUS_BAD_DEADZONE       // 死区异常信号，多为传感器变形
};

#if defined PROTO_HC

//#define DRIVER_SONAR_SELECT_MA40H1S
#define DRIVER_SONAR_SELECT_F401

#endif

#ifdef __cplusplus
}
#endif

#endif
