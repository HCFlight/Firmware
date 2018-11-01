/*************************************************************************
    @file: drv_f401.c
    @Author: li xiang
    @Mail: lixiang@zerozero.cn
    @Created Time: 2015/12/7 1:22:05
    @copyright Copyright (c) 2014-2015 Beijing Zero Zero Infinity Technology Co., Ltd all right reserved.
 ************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h> 
#include <px4_log.h>

#include <dev_fs_lib_spi.h>

//#include <fc/arch_api.h>
//#include <lib/log/log.h>

#include "drv_f401_static.h"

static uint8_t _image_buffer[8] = { 0 };
static int _image_size = 0;
static uint32_t _image_file_checksum= 0;

/**
 *@brief  Deinitializes f401 driver
 *@param  void
 *@retval int
 */
int f401_deinit(void)
{
    if (_fd > 0) {
        close(_fd);
        _fd = -1;
    }
    return F401_ERROR_CODE_OK;
}

/**
 *@brief  Initializes f401 driver
 *@param  const char* config
 *@retval int
 */
int f401_init(const char *config)
{
    if (_initiated)
        return F401_ERROR_CODE_OK;

    _fd = open(config, 0);
    if (_fd < 0) {
        PX4_ERR("HC_ERROR stm32f401_ drv_f401: error: failed to open spi device path: %s", config);
        return F401_ERROR_CODE_FAIL;
    }
    // Config SPI speed to 1 MHz
    struct dspal_spi_ioctl_set_bus_frequency freq;
    freq.bus_frequency_in_hz = F401_SPI10_FREQUENCY_1MHZ;
    if (ioctl(_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &freq) != 0) {
        PX4_ERR("HC_ERROR stm32f401_drv_f401: config spi speed failed\n");
        return F401_ERROR_CODE_FAIL;
    }

    pthread_attr_init(&_thread_attr);
    pthread_attr_setstacksize(&_thread_attr, F401_THREAD_STACK_SIZE);
    pthread_create(&_thread, &_thread_attr, _control_thread_main, 0);

    _initiated = true;

    return F401_ERROR_CODE_OK;
}

int f401_get_version(uint32_t *version)
{
    int error_code = _get_version();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    usleep(1000);
    *version = _version;
    return F401_ERROR_CODE_OK;
}

int f401_gimbal_follow_rad(uint8_t abs, float position, float speed)
{
    static struct _step_motor_param_s param;
    param.test = 0;
    param.abs = !!(abs);
    param.dir = (speed >= 0.0f) ? 1 : 0;
    param.reserved = 0;
    param.num_of_steps = _gimbal_position_convert(position);
    param.speed = _gimbal_speed_convert(speed);
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_POSITION, &param, sizeof(struct _step_motor_param_s));
}

int f401_gimbal_get_gear_gap(uint32_t *gear_gap)
{
    int error_code = _gimbal_get_gear_gap();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    *gear_gap = _gear_gap;
    return F401_ERROR_CODE_OK;
}

int f401_gimbal_get_raw(struct f401_gimbal_raw_s *raw)
{
    if (!_initiated)
        return F401_ERROR_CODE_UNINITIATED;
    if (_mode != _MODE_TYPE_APP)
        return F401_ERROR_CODE_MODE_NOT_APP;
    int error_code = _gimbal_get_sample();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    if (gimbal_status.status >= F401_GIMBAL_STATUS_MAX_NUM)
        return F401_ERROR_CODE_ENUM_UNMATCH;
    /*if (gimbal_status.selfcheck >= F401_GIMBAL_SELFCHECK_STATUS_MAX_NUM)
        return F401_ERROR_CODE_ENUM_UNMATCH;*/
    raw->cur_step            = gimbal_status.cur_step;
    raw->max_step            = gimbal_status.max_step;
    raw->cur_speed           = gimbal_status.cur_speed;
    raw->direction           = gimbal_status.direction;
    raw->cur_speed           = gimbal_status.cur_speed;
    raw->uplimit_triggered   = gimbal_status.uplimit;
    raw->downlimit_triggered = gimbal_status.downlimit;
    raw->status              = gimbal_status.status;
    raw->selfcheck_status    = gimbal_status.selfcheck;
    return F401_ERROR_CODE_OK;
}

int f401_gimbal_relocation(void)
{
    return _event_execute(_EVENT_TYPE_GIMBAL_RELOCATION, NULL, 0);
}

int f401_gimbal_reset(void)
{
    /*struct _step_motor_status_s s = {
        .status = F401_GIMBAL_STATUS_INIT_TO_UP
    };*/
    static struct _step_motor_param_s param = {
        .test = 1,
        .num_of_steps = 1,
        .speed = (F401_GIMBAL_MAX_SPEED << 1)
    };
    int error_code = _event_execute(_EVENT_TYPE_RESET_GIMBAL, &param, sizeof(struct _step_motor_param_s));
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    uint16_t delay = 0;
    while (gimbal_status.status != F401_GIMBAL_STATUS_INIT_WAIT_TIM) {
        if (delay > 2000)
            return F401_ERROR_CODE_TIMEOUT;
        //f401_gimbal_get_status(&s);
        usleep(1000);
        delay++;
    }
    return F401_ERROR_CODE_OK;
}

int f401_gimbal_set_gear_gap(uint32_t gear_gap)
{
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_GEAR_GAP, &gear_gap, sizeof(uint32_t));
}

int f401_gimbal_set_k(uint16_t k1, uint8_t k2)
{
    static struct _step_motor_ctrl_param_s param = { .k3 = 1 };
    param.k1 = k1;
    param.k2 = k2;
    //param.k3 = 1;
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_CTRL_PAR, &param, sizeof(struct _step_motor_ctrl_param_s));
}

int f401_gimbal_set_position(uint8_t reset, uint8_t abs, uint8_t dir, uint32_t step_num, uint16_t pulse_interval)
{
    /*struct _step_motor_status_s s = {
        .status = F401_GIMBAL_STATUS_INIT_UP_TO_DOWN
    };*/
    static struct _step_motor_param_s param = {
        .reserved = 1
    };
    param.test = !!(reset);
    param.abs = !!(abs);
    param.dir = !!(dir);
    param.num_of_steps = step_num;
    param.speed = pulse_interval;
    //param.reserved = 1;
    int error_code = _event_execute(_EVENT_TYPE_SET_GIMBAL_POSITION, &param, sizeof(struct _step_motor_param_s));
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    uint16_t delay = 0;
    while (gimbal_status.status != F401_GIMBAL_STATUS_RUN_WAIT_TIM) {
        if (delay > 2000)
            return F401_ERROR_CODE_TIMEOUT;
        //f401_gimbal_get_status(&s);
        usleep(1000);
        delay++;
    }
    return F401_ERROR_CODE_FAIL;
}

int f401_gimbal_set_position_rad(uint8_t abs, float position, float speed)
{
    static struct _step_motor_param_s param = {
        .test = 0,
        .reserved = 1
    };
    param.abs = !!(abs);
    param.dir = (speed >= 0.0f) ? 1 : 0;
    param.num_of_steps = _gimbal_position_convert(position) + F401_GIMBAL_OFFSET - F401_PITCH_OFFSET;
    param.speed = _gimbal_speed_convert(speed);
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_POSITION, &param, sizeof(struct _step_motor_param_s));
}

int f401_gimbal_set_speed(uint8_t direction, uint16_t speed)
{
    static struct _step_motor_param_s param = {
        .test = 0,
        .abs = 0,
        .num_of_steps = 0xFFFFFFFF,
        .reserved = 0
    };
    param.dir = !!direction;
    param.speed = speed;
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_SPEED, &param, sizeof(struct _step_motor_param_s));
}

int f401_gimbal_set_speed_rad(float speed)
{
    static struct _step_motor_param_s param = {
        .test = 0,
        .abs = 0,
        .num_of_steps = 0xFFFFFFFF,
        .reserved = 0
    };
    param.dir = (speed >= 0.0f) ? 1 : 0;
    param.speed = _gimbal_speed_convert(speed);
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_SPEED, &param, sizeof(struct _step_motor_param_s));
}

int f401_gimbal_start_follow(float pitch)
{
    static uint32_t pitch_init;
    pitch_init = _gimbal_position_convert(pitch);
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_INIT_POSITION, &pitch_init, sizeof(uint32_t));
}

int f401_gimbal_stop(void)
{
    static struct _step_motor_param_s param = {
        .test = 0,
        .abs = 0,
        .dir = 0,
        .num_of_steps = 0xFFFFFFFF,
        .speed = 0xFFFF
    };
    return _event_execute(_EVENT_TYPE_STOP_GIMBAL, &param, sizeof(struct _step_motor_param_s));
}

int f401_gimbal_stop_follow(void)
{
    int error_code = f401_gimbal_stop();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    return _event_execute(_EVENT_TYPE_SET_GIMBAL_STOP_FOLLOW, NULL, 0);
}

int f401_image_load(const uint8_t *buffer, int len)
{
    int loaded_size = len > F401_MAX_LOAD_SIZE ? F401_MAX_LOAD_SIZE : len;
    memcpy(_image_buffer + _image_size, buffer, loaded_size);
    _image_size += loaded_size;
    return loaded_size;
}

int f401_image_update(void)
{
    int loaded_size = 0;
    int total_load_times = (2 * _image_size) / F401_PACKAGE_SIZE;
    int current_load_times = 0;
    usleep(1000000);
    f401_power_led_set_freq(F401_POWER_LED_FREQ_UPDATING);
  //  log_printf("connect f401 spi.\n");
    if (_update_erase_memory(_image_size)) {
      //  log_printf("erase failed.\n");
        return F401_ERROR_CODE_FAIL;
    }
  //  log_printf("f401 flash erased.\n");
    while (loaded_size < _image_size) {
        if (current_load_times >= total_load_times)
            return F401_ERROR_CODE_TIMEOUT;
        loaded_size += _update_program(_image_buffer + loaded_size, _image_size - loaded_size);
        current_load_times++;
    }
    //log_printf("origin checksum is %x\n", _image_file_checksum);
    _update_get_checksum();
    usleep(10000);
    if (_image_file_checksum != _boot_checksum)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int f401_image_validate(void)
{
    _image_file_checksum = 0;
    for (int i = 0; i < _image_size; i++) {
        _image_file_checksum += _image_buffer[i];
    }
    return _image_file_checksum;
}

int f401_jump_to_app(void)
{
    uint32_t version = 0;
    uint16_t delay = 0;
    int error_code = _update_reset();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    usleep(180000); // wait for 401 jump to app
    while ((version & F401_APP_VERSION_HEAD) == 0) {
        if (delay > 1800)
            return F401_ERROR_CODE_TIMEOUT;
        if (_get_version() == F401_ERROR_CODE_OK) {
            usleep(1000);
            version = _version;
        }
        delay++;
    }
    _in_application();
    return F401_ERROR_CODE_OK;
}

int f401_jump_to_bootloader(void)
{
    uint32_t version = 0;
    uint16_t delay = 0;
    _image_size = 0;
    _in_bootloader();
    int error_code = _event_execute(_EVENT_TYPE_GO_BOOTLOADER, NULL, 0);
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    usleep(200000);
    while ((version & F401_BOOT_VERSION_HEAD) == 0) {
        if (delay > 2000) {
            _in_application();
            return F401_ERROR_CODE_TIMEOUT;
        }
        if (_get_version() == F401_ERROR_CODE_OK) {
            usleep(1000);
            version = _version;
        }
        delay++;
    }
    return F401_ERROR_CODE_OK;
}

int f401_power_led_set_freq(float freq)  // 闪烁频率（Hz）
{
    uint32_t index;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD,
        .length = F401_XCP_SEND_LENGTH,
        .cmd = F401_XCP_SET_LED
    };
    sendpackage.data[0] = (uint8_t)(freq * 4);
    int error_code = _event_post(&index, _EVENT_TYPE_UPDATE_SET_LED, (void *)&sendpackage, sizeof(struct f401_protocol_s), NULL);
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    return(_event_wait_finish(_EVENT_TYPE_UPDATE_SET_LED, index, WAIT_DELAY_TIME_US));
}

int f401_power_led_set_mode(uint8_t mode)
{
    static struct _power_led_param_s led_param;

    switch (mode) {
    case F401_BREATHLED_MODE_OFF:
    case F401_BREATHLED_MODE_ON:
    case F401_BREATHLED_MODE_BREATH:
        led_param.led_freq = 1000;
        break;
    case F401_BREATHLED_MODE_BLINK:
        led_param.led_freq = (uint16_t)(F401_POWER_LED_FREQ_NORMAL * 4);
        break;
    default:
        return F401_ERROR_CODE_ENUM_UNMATCH;
        break;
    }
    led_param.led_mode = mode;

    return _event_execute(_EVENT_TYPE_SET_BREATHLED, (void *)&led_param, sizeof(struct _power_led_param_s));
}

int f401_sonar_get_height(uint32_t *height)
{
    if (!_initiated)
        return F401_ERROR_CODE_UNINITIATED;
    *height = sonar_raw.distance_mm[0];
    return F401_ERROR_CODE_OK;
}

int f401_sonar_get_raw(struct sensor_sonar_raw_s *raw)
{
    if (!_initiated)
        return F401_ERROR_CODE_UNINITIATED;
    if (_mode != _MODE_TYPE_APP)
        return F401_ERROR_CODE_MODE_NOT_APP;
    int error_code = _sonar_get_sample();
    if (error_code != F401_ERROR_CODE_OK)
        return error_code;
    usleep(1000);   // 如果出问题就把这行反注释掉
    memcpy((void *)raw, (void *)&sonar_raw, sizeof(struct sensor_sonar_raw_s));

    return F401_ERROR_CODE_OK;
}

int f401_sonar_start(void)
{
    static uint8_t sonar_control = 1;
    return _event_execute(_EVENT_TYPE_SONAR_ENABLE, &sonar_control, sizeof(uint8_t));
}

int f401_sonar_stop(void)
{
    static uint8_t sonar_control = 0;
    return _event_execute(_EVENT_TYPE_SONAR_ENABLE, &sonar_control, sizeof(uint8_t));
}

int f401_sonar_set_pulse(uint8_t test_enable, uint16_t pulse_num)
{
    static struct _sonar_param_s param;
    param.test_enable = test_enable;
    param.num_pulse = pulse_num;
    if (param.num_pulse >= F401_MAX_SONAR_PULSE_NUM)
        param.num_pulse = F401_MAX_SONAR_PULSE_NUM;
    return _event_execute(_EVENT_TYPE_SET_SONAR_PAR, &param, sizeof(struct _sonar_param_s));
}

/*
static int32_t f401_get_sonar_par(struct _sonar_param_s *param)
{
    f401_getdata_package((uint8_t)F401_GET_SONAR_PAR, (void*)param, sizeof(struct _sonar_param_s));
    return F401_ERROR_CODE_OK;
}

int32_t f401_sonar_get_raw(unsigned int *framenum, unsigned int *height, unsigned char *sonarraw)
{
    struct f401_protocol_s sendprotocol = {
            .head = F401_MASTER_HEAD,
            .cmd = VERSIONID
    };
    //read sonarheight
    _sendcmd_package(&sendprotocol, (uint8_t)GETSONARRAW, NULL, 0);
    usleep(500);
    _read_reg(&sendprotocol, 512);
    memcpy(sonarraw, &_spi_rxbuf[0], 512);
    return F401_ERROR_CODE_OK;
}
*/

/*void f401_set_follow_constant(float pitch)
{
    struct _step_motor_status_s s;
    f401_gimbal_get_status(&s);
    follow_constant = s.cur_step + pitch + F401_GIMBAL_OFFSET_STEPS;
}

float f401_get_follow_constant(void)
{
    return follow_constant;
}*/
