#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h> 
#include <math.h>
#include <px4_log.h>

#include <dev_fs_lib_spi.h>

//#include <fc/arch_api.h>
#include "fc_define/arch_api.h"

#include "drv_f401_static.h"

bool _initiated = false;

enum _queue_status_e {
    _QUEUE_STATUS_FREE,
    _QUEUE_STATUS_RESERVE,
    _QUEUE_STATUS_COMMIT
};

struct event_queue_s {
    volatile uint32_t head;
    volatile uint32_t tail;
    uint32_t overflow;
    struct _event_s events[F401_QUEUE_SIZE];
};

static struct event_queue_s _queue;
static uint32_t _event_error[_EVENT_TYPE_MAX] = { 0 };      // 按位 1 表有错误，0 表没有错误
static uint32_t _event_pending[_EVENT_TYPE_MAX] = { 0 };    // 按位 1 表待处理，0 表无需处理或处理完成

enum _mode_type_e _mode = _MODE_TYPE_BOOT;
static const uint8_t _cmd_filter[_MODE_TYPE_MAX][_EVENT_TYPE_MAX] = {
    [_MODE_TYPE_APP] = {
        [_EVENT_TYPE_SET_BREATHLED]            = ENABLE,
        [_EVENT_TYPE_RESET_GIMBAL]             = ENABLE,
        [_EVENT_TYPE_STOP_GIMBAL]              = ENABLE,
        [_EVENT_TYPE_SET_SONAR_PAR]            = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_SPEED]         = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_POSITION]      = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_INIT_POSITION] = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_STOP_FOLLOW]   = ENABLE,
        [_EVENT_TYPE_GIMBAL_RELOCATION]        = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_CTRL_PAR]      = ENABLE,
        [_EVENT_TYPE_GET_SONAR_PAR]            = ENABLE,
        [_EVENT_TYPE_GET_GIMBAL_STATUS]        = ENABLE,
        [_EVENT_TYPE_GET_SONAR_HEIGHT]         = ENABLE,
        [_EVENT_TYPE_GET_F401_VERSION]         = ENABLE,
        [_EVENT_TYPE_GO_BOOTLOADER]            = ENABLE,
        [_EVENT_TYPE_SONAR_ENABLE]             = ENABLE,
        [_EVENT_TYPE_UPDATE_JUMP]              = ENABLE,
        [_EVENT_TYPE_UPDATE_SET_LED]           = DISABLE,
        [_EVENT_TYPE_UPDATE_SET_MTA]           = DISABLE,
        [_EVENT_TYPE_UPDATE_ERASE]             = DISABLE,
        [_EVENT_TYPE_UPDATE_PROGRAM]           = DISABLE,
        [_EVENT_TYPE_UPDATE_GET_CHECKSUM]      = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_GEAR_GAP]      = ENABLE,
        [_EVENT_TYPE_GET_GIMBAL_GEAR_GAP]      = ENABLE
    },
    [_MODE_TYPE_BOOT] = {
        [_EVENT_TYPE_SET_BREATHLED]            = DISABLE,
        [_EVENT_TYPE_RESET_GIMBAL]             = DISABLE,
        [_EVENT_TYPE_STOP_GIMBAL]              = DISABLE,
        [_EVENT_TYPE_SET_SONAR_PAR]            = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_SPEED]         = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_POSITION]      = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_INIT_POSITION] = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_STOP_FOLLOW]   = DISABLE,
        [_EVENT_TYPE_GIMBAL_RELOCATION]        = DISABLE,
        [_EVENT_TYPE_SET_GIMBAL_CTRL_PAR]      = DISABLE,
        [_EVENT_TYPE_GET_SONAR_PAR]            = DISABLE,
        [_EVENT_TYPE_GET_GIMBAL_STATUS]        = DISABLE,
        [_EVENT_TYPE_GET_SONAR_HEIGHT]         = DISABLE,
        [_EVENT_TYPE_GET_F401_VERSION]         = ENABLE,
        [_EVENT_TYPE_GO_BOOTLOADER]            = ENABLE,
        [_EVENT_TYPE_SONAR_ENABLE]             = DISABLE,
        [_EVENT_TYPE_UPDATE_JUMP]              = ENABLE,
        [_EVENT_TYPE_UPDATE_SET_LED]           = ENABLE,
        [_EVENT_TYPE_UPDATE_SET_MTA]           = ENABLE,
        [_EVENT_TYPE_UPDATE_ERASE]             = ENABLE,
        [_EVENT_TYPE_UPDATE_PROGRAM]           = ENABLE,
        [_EVENT_TYPE_UPDATE_GET_CHECKSUM]      = ENABLE,
        [_EVENT_TYPE_SET_GIMBAL_GEAR_GAP]      = DISABLE,
        [_EVENT_TYPE_GET_GIMBAL_GEAR_GAP]      = DISABLE
    }
};

int _fd = -1;

uint8_t _spi_rxbuf[F401_SPI_RECVBUF_LEN] = { 0 };
uint8_t _spi_txbuf[F401_SPI_RECVBUF_LEN] = { 0 };

static bool _queue_initiated = false;
uint32_t _boot_checksum = 0;
uint32_t _gear_gap = 0;
uint32_t _version = 0;
float follow_constant = 0.0f;
struct _step_motor_status_s gimbal_status;
struct sensor_sonar_raw_s sonar_raw;

pthread_t _thread = 0;
pthread_attr_t _thread_attr;

void *_control_thread_main(void *param)
{
    _init_queue();
    while (1)
        _control_loop();
}

void _control_loop(void)
{
    int result;
    uint32_t index;
    struct _event_s data;

    if (!_event_fetch(&data, &index))
        return;

    result = _dispatch_message(&data, index);
    if (data.call)
        data.call(result, index);
}

void _in_application(void)
{
    _mode = _MODE_TYPE_APP;
}

void _in_bootloader(void)
{
    _mode = _MODE_TYPE_BOOT;
}

void _init_queue(void)
{
    int i;
    _queue.head = 0;
    _queue.tail = 0;
    _queue.overflow = 0;
    for (i = 0; i < F401_QUEUE_SIZE; i++)
        _queue.events[i].ready = _QUEUE_STATUS_FREE;
    _queue_initiated = true;
}

// 判断数据有效性，如果所有字节相同则为异常数据
int _data_validate(const uint8_t *data, int length)
{
    int i;
    const uint8_t data0 = data[0];
    for (i = 1; i < length; i++)
        if (data[i] != data0)
            return F401_ERROR_CODE_OK;
    return F401_ERROR_CODE_INVALID_DATA;
}

int _dispatch_message(struct _event_s *data, uint32_t index)
{
    int result = F401_ERROR_CODE_FAIL;

    switch (data->type) {
    case _EVENT_TYPE_SET_BREATHLED:
        result = _led_control((struct _power_led_param_s *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_RESET_GIMBAL:
    case _EVENT_TYPE_STOP_GIMBAL:
    case _EVENT_TYPE_SET_GIMBAL_SPEED:
    case _EVENT_TYPE_SET_GIMBAL_POSITION:
        result = _gimbal_set_position((struct _step_motor_param_s *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_SET_SONAR_PAR:
        result = _sonar_set_parameters((struct _sonar_param_s *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_SET_GIMBAL_INIT_POSITION:
        result = _gimbal_set_init_step((uint32_t *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_SET_GIMBAL_STOP_FOLLOW:
        result = _gimbal_stop_follow();
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_GIMBAL_RELOCATION:
        result = _gimbal_relocation();
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_SET_GIMBAL_CTRL_PAR:
        result = _gimbal_set_control_parameters((struct _step_motor_ctrl_param_s *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_GET_SONAR_PAR:
        //result = f401_get_sonar_par(&sonar);
        break;
    case _EVENT_TYPE_GET_GIMBAL_STATUS:
        result = _gimbal_get_status();
        break;
    case _EVENT_TYPE_GET_SONAR_HEIGHT:
        result = _sonar_get_height();
        break;
    case _EVENT_TYPE_GET_F401_VERSION:
        result = _spi_get_version();
        break;
    case _EVENT_TYPE_GO_BOOTLOADER:
        result = _go_bootloader();
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_SONAR_ENABLE:
        result = _sonar_enable((unsigned char *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_UPDATE_JUMP:
    case _EVENT_TYPE_UPDATE_SET_LED:
    case _EVENT_TYPE_UPDATE_SET_MTA:
    case _EVENT_TYPE_UPDATE_ERASE:
        result = _update_write((struct f401_protocol_s *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_UPDATE_PROGRAM:
        result = _update_program_flash((struct f401_protocol_s *)(data->buf));
        break;
    case _EVENT_TYPE_UPDATE_GET_CHECKSUM:
        result = _update_write((struct f401_protocol_s *)(data->buf));
        break;
    case _EVENT_TYPE_SET_GIMBAL_GEAR_GAP:
        result = _gimbal_set_gear_gap((uint32_t *)(data->buf));
        _event_set_finished(data->type, index);
        break;
    case _EVENT_TYPE_GET_GIMBAL_GEAR_GAP:
        result = _spi_get_gear_gap();
        break;
    default:
        _event_set_finished(data->type, index);
        break;
    }
    return result;
}

int _event_execute(unsigned char f401_event, void *data, unsigned short datalen)
{
    uint32_t index;
    int error_code = _event_post(&index, f401_event, data, datalen, NULL);
    if (error_code != F401_ERROR_CODE_OK){
        PX4_INFO("stm32f401_ _event_excute _event_post failed");
        return error_code;
    }
    error_code = _event_wait_finish(f401_event, index, WAIT_DELAY_TIME_US);
    if (error_code != F401_ERROR_CODE_OK){
        PX4_INFO("stm32f401_ _event_excute _event_wait_finish failed");
        return error_code;
    }
    return F401_ERROR_CODE_OK;
}

int _event_fetch(struct _event_s *data, uint32_t *index)
{
    int head;
    uint32_t scan;

    for (scan = _queue.head; scan < _queue.tail; scan++) {
        struct _event_s *event = EVENT(&_queue, scan);
        int ready = event->ready;
        if (ready == _QUEUE_STATUS_RESERVE)
            return F401_ERROR_CODE_OK;
        if (ready == _QUEUE_STATUS_COMMIT && (arch_bool_compare_and_swap(&event->ready, _QUEUE_STATUS_COMMIT, _QUEUE_STATUS_FREE))) {
            *data = *event;
            *index = EVENT_IDX(scan);
            scan++;
            while (scan - (head = _queue.head) > 0) {
                if (arch_bool_compare_and_swap(&_queue.head, head, scan))
                    break;
            }
            return F401_ERROR_CODE_FAIL;
        }
    }
    return F401_ERROR_CODE_OK;
}

int _event_post(uint32_t *index, uint8_t type, void *arg, uint8_t len, operate_callback call)
{
    uint32_t scan;

    if (_queue_initiated == false || _cmd_filter[_mode][type] == DISABLE){
        PX4_INFO("stm32f401_: _event_post 285, _queue_initiated:%d, cmdFilter:%d _mode:%d, type:%d]\r\n", _queue_initiated,_cmd_filter[_mode][type],_mode, type);
        return F401_ERROR_CODE_FAIL;
    }

    for (scan = _queue.tail; scan - _queue.head < F401_QUEUE_SIZE - 1; scan++) {
        struct _event_s *event = EVENT(&_queue, scan);
        uint32_t tail;
        if (event->ready != _QUEUE_STATUS_FREE)
            continue;
        if (arch_bool_compare_and_swap(&event->ready, _QUEUE_STATUS_FREE, _QUEUE_STATUS_RESERVE)) {
            event->type = type;
            event->len = len;
            event->call = call;
            if (len > 0)
                memcpy(event->buf, arg, len);
            *index = EVENT_IDX(scan);
            _event_set_pending(type, *index);
            event->ready = _QUEUE_STATUS_COMMIT;
            scan++;
            while (scan - (tail = _queue.tail) > 0) {
                if (arch_bool_compare_and_swap(&_queue.tail, tail, scan))
                    break;
            }
           //  PX4_INFO("stm32f401_: return F401_ERROR_CODE_OK\r\n");
            return F401_ERROR_CODE_OK;
        }
    }
    /* event queue full */
    _queue.overflow++;
     PX4_INFO("stm32f401_: _event_post event queue full\r\n");
    return F401_ERROR_CODE_FAIL;
}

int _event_set_error(uint16_t type, uint32_t value)
{
    if (type >= _EVENT_TYPE_MAX)
        return F401_ERROR_CODE_ENUM_UNMATCH;
    _event_error[type] |= (1 << value);
    return F401_ERROR_CODE_OK;
}

int _event_set_finished(uint16_t type, uint32_t value)
{
    if (type >= _EVENT_TYPE_MAX)
        return F401_ERROR_CODE_ENUM_UNMATCH;
    _event_pending[type] &= ~(1 << value);
    return F401_ERROR_CODE_OK;
}

int _event_set_pending(uint16_t type, uint32_t value)
{
    if (type >= _EVENT_TYPE_MAX)
        return F401_ERROR_CODE_ENUM_UNMATCH;
    const uint32_t _mask = (1 << value);
    _event_error[type] &= (~_mask);
    _event_pending[type] |= _mask;
    return F401_ERROR_CODE_OK;
}

int _event_wait_finish(uint16_t type, uint32_t value, int32_t useconds)
{
    if (type >= _EVENT_TYPE_MAX)
        return F401_ERROR_CODE_ENUM_UNMATCH;
    const uint32_t _mask = (1 << value);
    do {
        if (_event_pending[type] & _mask) {
            usleep(1);  // 会不会太频繁了？
            continue;
        }
        if (_event_error[type] & _mask)
            return F401_ERROR_CODE_INVALID_DATA;
        return F401_ERROR_CODE_OK;
    } while (--useconds > 0);
    return F401_ERROR_CODE_TIMEOUT;
}

int _flash_write_delay(int result, uint32_t index)
{
    if (result)
        return F401_ERROR_CODE_FAIL;
    usleep(500);
    _event_set_finished(_EVENT_TYPE_UPDATE_PROGRAM, index);
    return F401_ERROR_CODE_OK;
}

int _get_version(void)
{
    int result;
    uint32_t index;
    result = _event_post(&index, _EVENT_TYPE_GET_F401_VERSION, NULL, 0, _read_version);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_GET_F401_VERSION, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_get_gear_gap(void)
{
    int result;
    uint32_t index;
    result = _event_post(&index, _EVENT_TYPE_GET_GIMBAL_GEAR_GAP, NULL, 0, _gimbal_read_gear_gap);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_GET_GIMBAL_GEAR_GAP, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_get_sample(void)
{
    int result;
    uint32_t index;
    result = _event_post(&index, _EVENT_TYPE_GET_GIMBAL_STATUS, NULL, 0, _gimbal_read_status);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_GET_GIMBAL_STATUS, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_get_status(void)
{
    if (_sendcmd_package(F401_GET_MOTOR_STATUS, NULL, 0) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_read_gear_gap(int result, uint32_t index)
{
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    usleep(200);
    _read_reg(&sendpackage, sizeof(uint32_t));
    const int data_validate_result = _data_validate(_spi_rxbuf, sizeof(uint32_t));
    if (data_validate_result == F401_ERROR_CODE_OK)
        memcpy((void *)&_gear_gap, (void *)_spi_rxbuf, sizeof(uint32_t));
    else
        _event_set_error(_EVENT_TYPE_GET_GIMBAL_GEAR_GAP, index);
    _event_set_finished(_EVENT_TYPE_GET_GIMBAL_GEAR_GAP, index);
    return data_validate_result;
}

int _gimbal_read_status(int result, uint32_t index)
{
    if (result)
        return F401_ERROR_CODE_FAIL;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    usleep(200);
    _read_reg(&sendpackage, sizeof(struct _step_motor_status_s));
    const int data_validate_result = _data_validate(_spi_rxbuf, sizeof(struct _step_motor_status_s));
    if (data_validate_result == F401_ERROR_CODE_OK)
        memcpy((void *)&gimbal_status, (void *)_spi_rxbuf, sizeof(struct _step_motor_status_s));
    else
        _event_set_error(_EVENT_TYPE_GET_GIMBAL_STATUS, index);
    _event_set_finished(_EVENT_TYPE_GET_GIMBAL_STATUS, index);
    return data_validate_result;
}

int _gimbal_relocation(void)
{
    if (_sendcmd_package((uint8_t)F401_MOTOR_RELOCATION, NULL, 0) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_set_control_parameters(struct _step_motor_ctrl_param_s *param)
{
    if (_sendcmd_package((uint8_t)F401_SET_MOTOR_CTRL_PAR, (void *)param, sizeof(struct _step_motor_ctrl_param_s)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_set_gear_gap(uint32_t *gear_gap)
{
    if (_sendcmd_package((uint8_t)F401_MOTOR_SET_GEAR_GAP, (void *)gear_gap, sizeof(uint32_t)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_set_init_step(uint32_t *pitch)
{
    if (_sendcmd_package((uint8_t)F401_SET_MOTOR_INIT_STEP, (void *)pitch, sizeof(uint32_t)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_set_position(struct _step_motor_param_s *param)
{
    if (_sendcmd_package((uint8_t)F401_SET_MOTOR_PAR, (void *)param, sizeof(struct _step_motor_param_s)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _gimbal_stop_follow(void)
{
    if (_sendcmd_package((uint8_t)F401_MOTOR_STOP_FOLLOW, NULL, 0) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _go_bootloader(void)
{
    if (_sendcmd_package((uint8_t)F401_GO_BOOTLOADER, NULL, 0) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _led_control(struct _power_led_param_s *param)
{
    if (_sendcmd_package((uint8_t)F401_SET_LED_PAR, (void *)param, sizeof(struct _power_led_param_s)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

/**
 *@brief  read register data from f401
 *@param  uint8_t reg_addr  read register data from addr to ..
 *        uint8_t *r_data  read data to *data pointer
 *        uint32_t len   the length of read data
 *@retval int
 */
int _read_reg(struct f401_protocol_s *cmd, int length)
{
    if (_fd <= 0) {
      PX4_ERR("drv_f401_static: f401_get_data called withtout opening dev");
        return F401_ERROR_CODE_FAIL;
    }
    _spi_bulk_read(cmd, length);
    return F401_ERROR_CODE_OK;
}

int _sendcmd_package(uint8_t cmd, void *data, uint16_t length)
{
    struct f401_protocol_s sendpackage;

    if (length > F401_SPI_SEND_MAX)
        return F401_ERROR_CODE_FAIL;
    sendpackage.head = F401_MASTER_HEAD;
    sendpackage.length = length + 1;
    sendpackage.cmd = cmd;
    if (length)
        memcpy((void*)&sendpackage.data[0], data, (length));
    if (_write_reg(&sendpackage, F401_SPI_CMD_LEN_MAX))
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _read_version(int result, uint32_t index)
{
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    usleep(200);
    _read_reg(&sendpackage, sizeof(uint32_t));
    const int data_validate_result = _data_validate(_spi_rxbuf, sizeof(uint32_t));
    if (data_validate_result == F401_ERROR_CODE_OK)
        memcpy((void *)&_version, (void *)_spi_rxbuf, sizeof(uint32_t));
    else
        _event_set_error(_EVENT_TYPE_GET_F401_VERSION, index);
    _event_set_finished(_EVENT_TYPE_GET_F401_VERSION, index);
    return data_validate_result;
}

int _spi_bulk_read(struct f401_protocol_s *cmd, int length)
{
    struct dspal_spi_ioctl_read_write read_write;

    read_write.write_buffer = _spi_txbuf;
    read_write.write_buffer_length = length;
    read_write.read_buffer = _spi_rxbuf;
    read_write.read_buffer_length = length;
    int ret = ioctl(_fd, SPI_IOCTL_RDWR, &read_write);

    return ret;
}

int _spi_get_gear_gap(void)
{
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    if (_sendcmd_package(F401_MOTOR_GET_GEAR_GAP, &sendpackage, (F401_XCP_SEND_LENGTH - 1)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _spi_get_version(void)
{
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    if (_sendcmd_package(F401_VERSION_ID, &sendpackage, (F401_XCP_SEND_LENGTH - 1)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _sonar_enable(uint8_t *param)
{
    if (_sendcmd_package((uint8_t)F401_SONAR_ENABLE, (void *)param, sizeof(uint8_t)) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _sonar_get_height(void)
{
    if (_sendcmd_package(F401_GET_SONAR_HEIGHT, NULL, 0) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _sonar_get_sample(void)
{
    int result;
    uint32_t index;
    result = _event_post(&index, _EVENT_TYPE_GET_SONAR_HEIGHT, NULL, 0, _sonar_read_height);
    if (result != F401_ERROR_CODE_OK)
        return result;
    if (_event_wait_finish(_EVENT_TYPE_GET_SONAR_HEIGHT, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _sonar_read_height(int result, uint32_t index)
{
    if (result)
        return F401_ERROR_CODE_FAIL;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    usleep(200);
    _read_reg(&sendpackage, sizeof(struct sensor_sonar_raw_s));
    const int data_validate_result = _data_validate(_spi_rxbuf, sizeof(struct sensor_sonar_raw_s));
    if (data_validate_result == F401_ERROR_CODE_OK)
        memcpy((void *)&sonar_raw, (void *)_spi_rxbuf, sizeof(struct sensor_sonar_raw_s));
    else
        _event_set_error(_EVENT_TYPE_GET_SONAR_HEIGHT, index);
    _event_set_finished(_EVENT_TYPE_GET_SONAR_HEIGHT, index);
    return data_validate_result;
}

int _sonar_set_parameters(struct _sonar_param_s *param)
{
    return _sendcmd_package((uint8_t)F401_SET_SONAR_PAR, (void*)param, sizeof(struct _sonar_param_s));
}

int _update_erase_memory(uint32_t len)
{
    int result;
    uint32_t index;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD,
        .length = F401_XCP_SEND_LENGTH,
        .cmd = F401_XCP_SET_MTA
    };
    sendpackage.data[0] = 0;
    sendpackage.data[1] = 0;
    sendpackage.data[2] = 0;
    sendpackage.data[3] = (uint8_t)F401_USER_PROGRAM_ADDR;
    sendpackage.data[4] = (uint8_t)(F401_USER_PROGRAM_ADDR >> 8);
    sendpackage.data[5] = (uint8_t)(F401_USER_PROGRAM_ADDR >> 16);
    sendpackage.data[6] = (uint8_t)(F401_USER_PROGRAM_ADDR >> 24);
    result = _event_post(&index, _EVENT_TYPE_UPDATE_SET_MTA, (void *)&sendpackage, sizeof(struct f401_protocol_s), NULL);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_UPDATE_SET_MTA, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;

    sendpackage.cmd = F401_XCP_PROGRAM_CLEAR;
    sendpackage.data[3] = (uint8_t)len;
    sendpackage.data[4] = (uint8_t)(len >> 8);
    sendpackage.data[5] = (uint8_t)(len >> 16);
    sendpackage.data[6] = (uint8_t)(len >> 24);
    result = _event_post(&index, _EVENT_TYPE_UPDATE_ERASE, (void *)&sendpackage, sizeof(struct f401_protocol_s), NULL);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_UPDATE_ERASE, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    usleep(5000000);
    return F401_ERROR_CODE_OK;
}

int _update_get_checksum(void)
{
    int result;
    uint32_t index;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD,
        .length = F401_XCP_SEND_LENGTH,
        .cmd = F401_XCP_GET_CHECKSUM
    };
    _boot_checksum = 0;
    result = _event_post(&index, _EVENT_TYPE_UPDATE_GET_CHECKSUM, (void *)&sendpackage, sizeof(struct f401_protocol_s), _update_read_checksum);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_UPDATE_GET_CHECKSUM, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _update_program(unsigned char* data, uint32_t len)
{
    int result;
    uint32_t index;
    int  program_size = (F401_XCP_SEND_LENGTH - 2) < len ? (F401_XCP_SEND_LENGTH - 2) : len;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD,
        .length = F401_XCP_SEND_LENGTH,
        .cmd = F401_XCP_PROGRAM
    };
    sendpackage.data[0] = program_size;
    memcpy(&sendpackage.data[1], data, program_size);

    result = _event_post(&index, _EVENT_TYPE_UPDATE_PROGRAM, (void *)&sendpackage, sizeof(struct f401_protocol_s), _flash_write_delay);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_OK;
    if (_event_wait_finish(_EVENT_TYPE_UPDATE_PROGRAM, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_OK;
    return program_size;
}

int _update_program_flash(struct f401_protocol_s *cmd)
{
    if (_update_write(cmd))
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _update_read_checksum(int result, uint32_t index)
{
    if (result)
        return F401_ERROR_CODE_FAIL;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD
    };
    usleep(1000);
    _read_reg(&sendpackage, sizeof(uint32_t));
    memcpy((void *)&_boot_checksum, (void *)_spi_rxbuf, sizeof(uint32_t));
    _event_set_finished(_EVENT_TYPE_UPDATE_GET_CHECKSUM, index);
    return F401_ERROR_CODE_OK;
}

int _update_reset(void)
{
    int result;
    uint32_t index;
    struct f401_protocol_s sendpackage = {
        .head = F401_MASTER_HEAD,
        .length = F401_XCP_SEND_LENGTH,
        .cmd = F401_XCP_PROGRAM_RESET
    };
    result = _event_post(&index, _EVENT_TYPE_UPDATE_JUMP, (void *)&sendpackage, sizeof(struct f401_protocol_s), NULL);
    if (result != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    if (_event_wait_finish(_EVENT_TYPE_UPDATE_JUMP, index, WAIT_DELAY_TIME_US) != F401_ERROR_CODE_OK)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

int _update_write(struct f401_protocol_s *cmd)
{
    uint8_t num;
    if (_fd <= 0) {
      PX4_ERR("drv_f401_static: mpu_spi_set_reg called withtout opening dev");
        return F401_ERROR_CODE_FAIL;
    }
    for (num = 0; num < F401_SPI_CMD_LEN_MAX; num++) {
        write(_fd, (((char*)cmd) + num), 1);
    }
    return F401_ERROR_CODE_OK;
}

/**
 *@brief  write register data from f401
 *@param  uint8_t reg_addr  write register data from addr to ..
 *        uint8_t *w_data  write data to *data pointer
 *        uint32_t len   the length of write data
 *@retval int
 */
int _write_reg(struct f401_protocol_s *cmd, uint8_t length)
{
    if (_fd <= 0) {
      PX4_ERR("drv_f401_static: mpu_spi_set_reg called withtout opening dev");
        return F401_ERROR_CODE_FAIL;
    }
    if (write(_fd, (char*)cmd, length) != length)
        return F401_ERROR_CODE_FAIL;
    return F401_ERROR_CODE_OK;
}

uint16_t _gimbal_speed_convert(float speed)
{
    float temp_r = fabsf(speed);
    if (temp_r < F401_GIMBAL_MIN_SPEED_RAD)
        return 0xFFFF;
    if (temp_r < F401_GIMBAL_STARTUP_SPEED_RAD)
        return (uint16_t)((F401_GIMBAL_TIMER_CONVERT_PAR / F401_GIMBAL_STARTUP_SPEED_RAD) - 1);
    if (temp_r > F401_GIMBAL_MAX_SPEED_RAD)
        temp_r = F401_GIMBAL_MAX_SPEED_RAD;
    return (uint16_t)((F401_GIMBAL_TIMER_CONVERT_PAR / temp_r) - 1); //((GIMBAL TIMER CLK * PI) /(47 * 10 * 32 * temp_r))
}

uint32_t _gimbal_position_convert(float pos)
{
    if (fabsf(pos) <= F401_GIMBAL_RAD_PER_PULSE)
        return F401_PITCH_OFFSET;
    else
        return (uint32_t)((pos / F401_GIMBAL_RAD_PER_PULSE) + F401_PITCH_OFFSET);
}
