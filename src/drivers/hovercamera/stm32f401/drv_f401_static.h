// drv_f401_static.h
// Jin Pengxiang
// Created on 2017/4/12

#ifndef _HEXAGON__DRIVER__F401_STATIC_H_
#define _HEXAGON__DRIVER__F401_STATIC_H_

#include <fcntl.h>
#include <pthread.h>

#include "drv_f401.h"

#define F401_MAX_LOAD_SIZE  256
#define F401_PACKAGE_SIZE   11

#define F401_QUEUE_BIT      4
#define F401_QUEUE_SIZE     (1 << F401_QUEUE_BIT)
#define F401_QUEUE_MASK     (F401_QUEUE_SIZE - 1)
#define F401_BUFFER_LENGTH  30

#define EVENT_IDX(idx)      ((idx) & F401_QUEUE_MASK)
#define EVENT(q, idx)       ((q)->events + EVENT_IDX(idx))
#define EVENT_FLAG(ev)      (1 << (ev))

#define ENABLE              1
#define DISABLE             0
#define WAIT_DELAY_TIME_US  10000

extern bool _initiated;

extern int _fd;

extern uint8_t _spi_rxbuf[F401_SPI_RECVBUF_LEN];
extern uint8_t _spi_txbuf[F401_SPI_RECVBUF_LEN];

extern uint32_t _boot_checksum;
extern uint32_t _gear_gap;
extern uint32_t _version;

extern float follow_constant;

extern pthread_t _thread;
extern pthread_attr_t _thread_attr;

extern struct _step_motor_status_s gimbal_status;
extern struct sensor_sonar_raw_s sonar_raw;

typedef int (*operate_callback)(int result, uint32_t index);

enum _event_type_e {
    _EVENT_TYPE_SET_BREATHLED = 0,
    _EVENT_TYPE_RESET_GIMBAL,
    _EVENT_TYPE_STOP_GIMBAL,
    _EVENT_TYPE_SET_SONAR_PAR,
    _EVENT_TYPE_SET_GIMBAL_SPEED,
    _EVENT_TYPE_SET_GIMBAL_POSITION,
    _EVENT_TYPE_SET_GIMBAL_INIT_POSITION,
    _EVENT_TYPE_SET_GIMBAL_STOP_FOLLOW,
    _EVENT_TYPE_GIMBAL_RELOCATION,
    _EVENT_TYPE_SET_GIMBAL_CTRL_PAR,
    _EVENT_TYPE_GET_SONAR_PAR,
    _EVENT_TYPE_GET_GIMBAL_STATUS,
    _EVENT_TYPE_GET_SONAR_HEIGHT,
    _EVENT_TYPE_GET_F401_VERSION,
    _EVENT_TYPE_GO_BOOTLOADER,
    _EVENT_TYPE_SONAR_ENABLE,
    _EVENT_TYPE_UPDATE_JUMP,
    _EVENT_TYPE_UPDATE_SET_LED,
    _EVENT_TYPE_UPDATE_SET_MTA,
    _EVENT_TYPE_UPDATE_ERASE,
    _EVENT_TYPE_UPDATE_PROGRAM,
    _EVENT_TYPE_UPDATE_GET_CHECKSUM,
    _EVENT_TYPE_SET_GIMBAL_GEAR_GAP,
    _EVENT_TYPE_GET_GIMBAL_GEAR_GAP,
    _EVENT_TYPE_MAX
};

enum _mode_type_e {
    _MODE_TYPE_APP,
    _MODE_TYPE_BOOT,
    _MODE_TYPE_MAX
};
extern enum _mode_type_e _mode;

struct _event_s {
    uint8_t type;
    uint8_t len;
    uint32_t ready;
    operate_callback call;
    uint8_t buf[F401_BUFFER_LENGTH];
};

struct _power_led_param_s {
    uint8_t led_mode;   // enum f401_breathled_mode_e
    uint16_t led_freq;
};

struct _step_motor_ctrl_param_s {
    uint16_t k1;
    uint8_t k2;
    uint8_t k3;
};

struct _step_motor_param_s {
    struct {
        uint8_t test : 1;
        uint8_t abs : 1;
        uint8_t dir : 1;
        uint8_t reserved : 5;
    };
    uint32_t num_of_steps;
    uint16_t speed;
};

struct _step_motor_status_s {
    uint32_t cur_step;
    uint32_t max_step;
    uint16_t cur_speed;
    struct {
        uint16_t direction : 1;
        uint16_t uplimit : 1;
        uint16_t downlimit : 1;
        uint16_t status : 4;
        uint16_t selfcheck : 2;
        uint16_t res : 7;
    };
};

struct _sonar_param_s {
    uint8_t test_enable;
    uint16_t num_pulse;
};

void *_control_thread_main(void *param);
void _control_loop(void);
void _in_application(void);
void _in_bootloader(void);
void _init_queue(void);

int _data_validate(const uint8_t *data, int length);
int _dispatch_message(struct _event_s *data, uint32_t index);
int _event_execute(unsigned char f401_event, void *data, unsigned short datalen);
int _event_fetch(struct _event_s *data, uint32_t *index);
int _event_post(uint32_t *index, uint8_t type, void *arg, uint8_t len, operate_callback call);
int _event_set_error(uint16_t type, uint32_t value);
int _event_set_finished(uint16_t type, uint32_t value);
int _event_set_pending(uint16_t type, uint32_t value);
int _event_wait_finish(uint16_t type, uint32_t value, int32_t useconds);
int _flash_write_delay(int result, uint32_t index);
int _get_version(void);
int _gimbal_get_gear_gap(void);
int _gimbal_get_sample(void);
int _gimbal_get_status(void);
int _gimbal_read_gear_gap(int result, uint32_t index);
int _gimbal_read_status(int result, uint32_t index);
int _gimbal_relocation(void);
int _gimbal_set_control_parameters(struct _step_motor_ctrl_param_s *param);
int _gimbal_set_gear_gap(uint32_t *gear_gap);
int _gimbal_set_init_step(uint32_t *pitch);
int _gimbal_set_position(struct _step_motor_param_s *param);
int _gimbal_stop_follow(void);
int _go_bootloader(void);
int _led_control(struct _power_led_param_s *param);
int _read_reg(struct f401_protocol_s *cmd, int length);
int _sendcmd_package(uint8_t cmd, void *data, uint16_t length);
int _read_version(int result, uint32_t index);
int _spi_bulk_read(struct f401_protocol_s *cmd, int length);
int _spi_get_gear_gap(void);
int _spi_get_version(void);
int _sonar_enable(uint8_t *param);
int _sonar_get_height(void);
int _sonar_get_sample(void);
int _sonar_read_height(int result, uint32_t index);
int _sonar_set_parameters(struct _sonar_param_s *param);
int _update_erase_memory(uint32_t len);
int _update_get_checksum(void);
int _update_program(unsigned char* data, uint32_t len);
int _update_program_flash(struct f401_protocol_s *cmd);
int _update_read_checksum(int result, uint32_t index);
int _update_reset(void);
int _update_write(struct f401_protocol_s *cmd); // What the fuck?
int _write_reg(struct f401_protocol_s *cmd, uint8_t length);

uint16_t _gimbal_speed_convert(float speed);

uint32_t _gimbal_position_convert(float pos);

#endif
