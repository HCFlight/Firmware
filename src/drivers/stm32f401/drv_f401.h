/*************************************************************************
    @file: drv_401.h
    @Author: li xiang
    @Mail: lixiang@zerozero.cn
    @Created Time: 2015/12/7 1:22:30
    @copyright Copyright (c) 2014-2015 Beijing Zero Zero Infinity Technology Co., Ltd all right reserved.
 ************************************************************************/
#ifndef _HEXAGON__DRIVER__F401_H_
#define _HEXAGON__DRIVER__F401_H_

enum f401_error_code_e {
    F401_ERROR_CODE_OK,
    F401_ERROR_CODE_FAIL,
    F401_ERROR_CODE_ENUM_UNMATCH,
    F401_ERROR_CODE_INVALID_DATA,
    F401_ERROR_CODE_MODE_NOT_APP,
    F401_ERROR_CODE_MODE_NOT_BOOT,
    F401_ERROR_CODE_TIMEOUT,
    F401_ERROR_CODE_UNINITIATED
};

//#include <station/defines/f401/f401.h>
#include "station_define/f401.h"

//#include <fc/basic_type.h>
#include "fc_define/basic_type.h"

//#include <fc/sensor/sonar.h>
#include "fc_define/sonar.h"

//#include <lib/math/math.h>

#define F401_THREAD_STACK_SIZE          (100 * 1024)
#define F401_SPI_PATH                   "/dev/spi-10"

#define F401_SPI10_FREQUENCY_1MHZ       1 * 1000 * 1000
#define F401_SPI10_FREQUENCY_10MHZ      10 * 1000 * 1000
#define F401_SPI10_FREQUENCY_15MHZ      15 * 1000 * 1000
#define F401_SPI10_FREQUENCY_20MHZ      20 * 1000 * 1000
#define F401_SPI_RECVBUF_LEN            2048

/*Command define***********************************************************/
#define F401_MASTER_HEAD                0xAA
#define F401_VERSION_ID                 0x00
#define F401_GET_SONAR_PAR              0x10
#define F401_SET_SONAR_PAR              0x11
#define F401_GET_SONAR_HEIGHT           0x12
#define F401_GET_SONAR_RAW              0x13
#define F401_SONAR_ENABLE               0x14
#define F401_GET_MOTOR_STATUS           0x20
#define F401_SET_MOTOR_PAR              0x21
#define F401_SET_MOTOR_CTRL_PAR         0x22
#define F401_SET_MOTOR_INIT_STEP        0x23
#define F401_MOTOR_RELOCATION           0x24
#define F401_MOTOR_STOP_FOLLOW          0x25
#define F401_MOTOR_SET_GEAR_GAP         0x26
#define F401_MOTOR_GET_GEAR_GAP         0x27
#define F401_SET_LED_PAR                0x31
#define F401_GO_BOOTLOADER              0xBE

#define F401_SPI_SEND_MAX               250
#define F401_SPI_CMD_LEN_MAX            15
#define F401_XCP_SEND_LENGTH            (F401_SPI_CMD_LEN_MAX - 2)

#define SONAR_SAMPLE_MAX                10240
#define F401_GIMBAL_MAX_SPEED           54
#define F401_POWER_LED_FREQ_NORMAL      0.75f
#define F401_POWER_LED_FREQ_UPDATING    2.5f

/*Firmware update Command define*******************************************/
#define F401_XCP_PROGRAM                0xF0
#define F401_XCP_PROGRAM_CLEAR          0xF1
#define F401_XCP_GET_CHECKSUM           0xF2
#define F401_XCP_SET_LED                0xF3
#define F401_XCP_PROGRAM_RESET          0xF4
#define F401_XCP_SET_MTA                0xF6
#define F401_XCP_CONNECT                0xFC

#define F401_UPDATE_SEND_MAX            64

#define F401_USER_PROGRAM_ADDR          0x8010000
#define F401_BOOT_VERSION_HEAD          0x10000000
#define F401_APP_VERSION_HEAD           0x40000000

#define F401_PI                         M_PI
#define F401_GIMBAL_MAX_DISTANCE        1.86f
#define F401_GIMBAL_MAX_SPEED_RAD       5.347f
#define F401_GIMBAL_MIN_SPEED_RAD       0.0042f
#define F401_GIMBAL_STARTUP_SPEED_RAD   0.1f
#define F401_PITCH_OFFSET               50000
#define F401_GIMBAL_OFFSET              20000000

#define F401_MAX_SONAR_PULSE_NUM        40

struct f401_protocol_s {
    uint8_t head;
    uint8_t length;
    uint8_t cmd;
    uint8_t data[F401_SPI_CMD_LEN_MAX];
};

struct f401_update_protocol_s {
    uint8_t length;
    uint8_t data[F401_UPDATE_SEND_MAX];
};

#define F401_SPI_CFG_FIELDS F401_SPI_PATH

int f401_deinit(void);
int f401_init(const char *config);

int f401_get_version(uint32_t *version);

int f401_gimbal_follow_rad(uint8_t abs, float position, float speed);
int f401_gimbal_get_gear_gap(uint32_t *gear_gap);
int f401_gimbal_get_raw(struct f401_gimbal_raw_s *raw);
int f401_gimbal_relocation(void);
int f401_gimbal_reset(void);
int f401_gimbal_set_gear_gap(uint32_t gear_gap);
int f401_gimbal_set_k(uint16_t k1, uint8_t k2);
int f401_gimbal_set_position(uint8_t reset, uint8_t abs, uint8_t dir, uint32_t step_num, uint16_t pulse_interval);
int f401_gimbal_set_position_rad(uint8_t abs, float position, float speed);
int f401_gimbal_set_speed(uint8_t direction, uint16_t speed);
int f401_gimbal_set_speed_rad(float speed);
int f401_gimbal_start_follow(float pitch);
int f401_gimbal_stop(void);
int f401_gimbal_stop_follow(void);

int f401_image_load(const uint8_t *buffer, int len);
int f401_image_update(void);
int f401_image_validate(void);

int f401_jump_to_app(void);
int f401_jump_to_bootloader(void);

int f401_power_led_set_freq(float freq);    // 闪烁频率（Hz）
int f401_power_led_set_mode(uint8_t mode);

int f401_sonar_get_height(uint32_t *height);
int f401_sonar_get_raw(struct sensor_sonar_raw_s *raw);
int f401_sonar_start(void);
int f401_sonar_stop(void);
int f401_sonar_set_pulse(uint8_t test_enable, uint16_t pulse_num);

#endif
