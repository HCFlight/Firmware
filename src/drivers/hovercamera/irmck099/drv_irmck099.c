/*************************************************************************
        @file: drv_motor_foc.c
        @Author: huang Pengfei
        @Mail: huangpengfei@zerozero.cn
        @Created Time: 2016/5/3 15:51:04
        @copyright Copyright (c) 2014-2015 Beijing Zero Zero Infinity Technology Co., Ltd all right reserved.
 ************************************************************************/

#include "fc_define/basic_type.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fcntl.h>

#include <sys/ioctl.h>      

#include <px4_log.h>

#include "drv_irmck099.h"


#ifdef DRIVER_MOTOR_SELECT_IRMCK099

//#include <lib/log/log.h>

#if !defined __GNUC__
#define __attribute__(...)
#endif

//BYTE0表示变量dwTemp的低8位
//BYTE1表示变量dwTemp的高8位
//串口发送16位数据，先发送高8位
#define BYTE0(dwTemp)   ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)   ( *( (char *)(&dwTemp) + 1) )

static int _fd = -1;
static int _mode = IRMCK099_MODE_STANDBY;
static uint8_t _tx_buf[20];

struct irmck099_motor_data_s _motor_data[IRMCK099_MOTOR_NUM] = {
    [0 ... IRMCK099_MOTOR_NUM - 1].speed = 0,
    [0 ... IRMCK099_MOTOR_NUM - 1].current = 0,
    [0 ... IRMCK099_MOTOR_NUM - 1].fault_reg = 0x0000
};

//修改irmck099芯片寄存器成功后，芯片会回复一条串口信息
//信息中包括节点地址，修改寄存器地址，修改的值
//通过这两个值，判断是否修改寄存器成功
static uint16_t _reg_addr_for_check = 0x0000;
static uint16_t _reg_val_for_check  = 0x0000;

struct irmck099_tx_std_msg_s _tx_std_msg = {
    .data_word0 = 0x00,
    .data_word1 = 0x00
};

struct irmck099_tx_long_msg_s _tx_long_msg = {
    .data_word0 = 0x00,
    .data_word1 = 0x00,
    .data_word2 = 0x00,
    .data_word3 = 0x00
};

static int _set_speed(const uint8_t motor_idx, int16_t motor_speed) __attribute__((unused));
static int _read_fault_flg(uint8_t addr) __attribute__((unused));
static int _get_fault_flg(uint8_t motor_idx, uint8_t *fault_flg) __attribute__((unused));
static int _which_fault(uint16_t motor_fault_flg, uint8_t *fault_flg);
static int16_t _std_msg_sum(struct irmck099_tx_std_msg_s *tx_std_msg);
static int16_t _long_msg_sum(struct irmck099_tx_long_msg_s *tx_long_msg);
static int _transmit_std_msg_prepare(void);
static int _transmit_long_msg_prepare(void);
static int _write_data(uint8_t *w_data, uint32_t write_size);
static int16_t _check_msg_sum(char *buffer, size_t num_bytes);
static int _receive_analysis(char *buffer, size_t num_bytes);
static int _write_register_with_check(uint8_t addr, uint16_t reg_addr, uint16_t reg_val);
static int _write_register(uint8_t addr, uint16_t reg_addr, uint16_t reg_val);
static int _write_register_check(uint16_t reg_addr, uint16_t reg_val);
static int _music_mode_entry(uint8_t addr);
static int _music_sound(uint8_t tone, uint32_t audio_time, uint32_t pause_time);
static int _music_mode_exit(uint8_t addr);
static int _modify_parameter(void);

void _recv_callback(void *context, char *buffer, size_t num_bytes);

static struct irmck099_config_s _cfg = {
    .receive_callback.rx_data_callback_func_ptr = _recv_callback,
    .irmck099_uart_path                         = IRMCK099_UART_PATH,
    .irmck099_uart_freq                         = { DSPAL_SIO_BITRATE_115200 }
};

int irmck099_motor_init(void)
{
    int result = 0;
    _fd = open(_cfg.irmck099_uart_path, 2);
    if (_fd < 0){
        PX4_INFO("irmck099_init open %s faild, result:%d", _cfg.irmck099_uart_path, _fd);
        return FC_FAIL;
    }

    result = ioctl(_fd, SERIAL_IOCTL_SET_DATA_RATE, &_cfg.irmck099_uart_freq);
    if (result < 0){

        PX4_INFO("irmck099_init setup freq, result:%d", _cfg.irmck099_uart_freq, result);
        return FC_FAIL;
    }

    result = ioctl(_fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&(_cfg.receive_callback));
    if (result < 0){
        PX4_INFO("irmck099_init setup callback result:%d", result);
        return FC_FAIL;
    }

    result = _modify_parameter();
    if (result < 0) {
        PX4_INFO("irmck099_init modify_parmeter:%d", result);
        return FC_FAIL;
    }
    _mode = IRMCK099_MODE_MOTOR_RUN;

    return result;
}

int irmck099_motor_deinit(void)
{
    if (_fd > 0) {
        close(_fd);
        _fd = -1;
        return FC_OK;
    }

    return FC_FAIL;
}

//设置四个电机速度set-point
int irmck099_set_speed_four_motor(int16_t motor_speed[IRMCK099_MOTOR_NUM])
{
    static uint8_t motor_idx_polling = 0;

    if (_mode == IRMCK099_MODE_MOTOR_RUN) {
        motor_idx_polling++;

        _tx_long_msg.len = IRMCK099_LONG_MSG_LEN;
        _tx_long_msg.node_addr = IRMCK099_BOARDCAST_ADDR;

        switch (motor_idx_polling % IRMCK099_MOTOR_NUM) {
        case 0:
            _tx_long_msg.cmd_num = IRMCK099_BOARDCAST_CHANNEL0_FLAG;
            break;
        case 1:
            _tx_long_msg.cmd_num = IRMCK099_BOARDCAST_CHANNEL1_FLAG;
            break;
        case 2:
            _tx_long_msg.cmd_num = IRMCK099_BOARDCAST_CHANNEL2_FLAG;
            break;
        case 3:
            _tx_long_msg.cmd_num = IRMCK099_BOARDCAST_CHANNEL3_FLAG;
            break;
        default:
            return FC_FAIL;
        }

        _tx_long_msg.data_word0 = (int16_t)((float)motor_speed[IRMCK099_CHANNEL0_MOTOR_INDEX] * IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE);
        _tx_long_msg.data_word1 = (int16_t)((float)motor_speed[IRMCK099_CHANNEL1_MOTOR_INDEX] * IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE);
        _tx_long_msg.data_word2 = (int16_t)((float)motor_speed[IRMCK099_CHANNEL2_MOTOR_INDEX] * IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE);
        _tx_long_msg.data_word3 = (int16_t)((float)motor_speed[IRMCK099_CHANNEL3_MOTOR_INDEX] * IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE);

        _tx_long_msg.check_sum = _long_msg_sum(&_tx_long_msg);
        _transmit_long_msg_prepare();

        _write_data(_tx_buf, _tx_long_msg.len);
    } else {
        return FC_FAIL;
    }

    return FC_OK;
}

int irmck099_get_motor_data(struct irmck099_motor_data_s *motor_data[IRMCK099_MOTOR_NUM])
{
    memcpy((void *)motor_data, (void *)&_motor_data, sizeof(_motor_data));

    return FC_OK;
}

int irmck099_get_speed(int16_t motor_speed[IRMCK099_MOTOR_NUM])
{
    motor_speed[0] = _motor_data[0].speed;
    motor_speed[1] = _motor_data[1].speed;
    motor_speed[2] = _motor_data[2].speed;
    motor_speed[3] = _motor_data[3].speed;

    return FC_OK;
}

int irmck099_get_current(uint16_t motor_current[IRMCK099_MOTOR_NUM])
{
    motor_current[0] = _motor_data[0].current;
    motor_current[1] = _motor_data[1].current;
    motor_current[2] = _motor_data[2].current;
    motor_current[3] = _motor_data[3].current;

    return FC_OK;
}

int irmck099_get_fault(uint16_t motor_fault_flag[IRMCK099_MOTOR_NUM])
{
    motor_fault_flag[0] = _motor_data[0].fault_reg;
    motor_fault_flag[1] = _motor_data[1].fault_reg;
    motor_fault_flag[2] = _motor_data[2].fault_reg;
    motor_fault_flag[3] = _motor_data[3].fault_reg;

    return FC_OK;
}

//发送读取错误寄存器命令
static int _read_fault_flg(uint8_t addr)
{
    _tx_std_msg.len = IRMCK099_STD_MSG_LEN;
    _tx_std_msg.node_addr = addr;
    _tx_std_msg.cmd_num = IRMCK099_READ_STATUS;

    _tx_std_msg.data_word0 = IRMCK099_READ_FAULT_FLG;
    _tx_std_msg.check_sum = _std_msg_sum(&_tx_std_msg);
    _transmit_std_msg_prepare();

    _write_data(_tx_buf, _tx_std_msg.len);

    return FC_OK;
}

static int _get_fault_flg(uint8_t motor_idx, uint8_t *fault_flg)
{
    if (motor_idx < IRMCK099_MOTOR_NUM) {
        _which_fault(_motor_data[motor_idx].fault_reg, fault_flg);
        return FC_OK;
    }

    return FC_FAIL;
}

int irmck099_play_music(uint8_t index)
{
    static uint8_t _music_played = 0;
    if (index == DYNAMIC_MOTOR_MUSIC_INDEX_POWER_ON && _music_played){
        return FC_FAIL;
    }
    int result = 0;

    _mode = IRMCK099_MODE_PLAY_MUSIC;

    result |= _music_mode_entry(IRMCK099_MOTOR0_ADDR);
    result |= _music_mode_entry(IRMCK099_MOTOR1_ADDR);
    result |= _music_mode_entry(IRMCK099_MOTOR2_ADDR);
    result |= _music_mode_entry(IRMCK099_MOTOR3_ADDR);

    switch (index) {
    case DYNAMIC_MOTOR_MUSIC_INDEX_POWER_ON:
        _music_played = 1;
        _music_sound(IRMCK099_DO, IRMCK099_SOUND_STD, IRMCK099_PAUSE_STD);
        _music_sound(IRMCK099_SO, IRMCK099_SOUND_STD, IRMCK099_PAUSE_STD);
        break;
    case DYNAMIC_MOTOR_MUSIC_INDEX_POWER_OFF:
        _music_sound(IRMCK099_SO, IRMCK099_SOUND_STD, IRMCK099_PAUSE_STD);
        _music_sound(IRMCK099_DO, IRMCK099_SOUND_STD, IRMCK099_PAUSE_STD);
        break;
    default:
        return FC_FAIL;
    }

    result |= _music_mode_exit(IRMCK099_MOTOR0_ADDR);
    result |= _music_mode_exit(IRMCK099_MOTOR1_ADDR);
    result |= _music_mode_exit(IRMCK099_MOTOR2_ADDR);
    result |= _music_mode_exit(IRMCK099_MOTOR3_ADDR);

    if (result == 0) {
        _mode = IRMCK099_MODE_MOTOR_RUN;
    }

    return result;
}

//串口回调函数，解析电调芯片irmck099返回来的数据
void _recv_callback(void *context, char *buffer, size_t num_bytes)
{
    // PX4_INFO("FOC did receive buffer: %d", num_bytes);
    // printUint8Buffer((uint8_t *)buffer, num_bytes);
    _receive_analysis(buffer, num_bytes);
}

static int _write_data(uint8_t *w_data, uint32_t write_size)
{
    uint32_t _ret = 0;

    if (_fd < 0)
        return FC_FAIL;

    _ret = write(_fd, (char *)w_data, write_size);
    if (_ret != write_size){
        return FC_FAIL;
    }
    // PX4_INFO("FOC did write buffer: %d", write_size);
    // printUint8Buffer(w_data, write_size);

    return FC_OK;
}

//内部测试：设置单个电机转速set-point
static int _set_speed(const uint8_t motor_idx, int16_t motor_speed)
{
    if (motor_idx >= IRMCK099_MOTOR_NUM)
        return FC_FAIL;

    _tx_std_msg.len = IRMCK099_STD_MSG_LEN;
    _tx_std_msg.cmd_num = IRMCK099_SET_ONE_SPEED;

    switch (motor_idx) {
    case 0:
        _tx_std_msg.node_addr = IRMCK099_MOTOR0_ADDR;
        break;
    case 1:
        _tx_std_msg.node_addr = IRMCK099_MOTOR1_ADDR;
        break;
    case 2:
        _tx_std_msg.node_addr = IRMCK099_MOTOR2_ADDR;
        break;
    case 3:
        _tx_std_msg.node_addr = IRMCK099_MOTOR3_ADDR;
        break;
    default:
        return FC_FAIL;
    }

    _tx_std_msg.data_word1 = (int16_t)((float)motor_speed * IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE);
    _tx_std_msg.check_sum = _std_msg_sum(&_tx_std_msg);
    _transmit_std_msg_prepare();

    _write_data(_tx_buf, _tx_std_msg.len);

    return FC_OK;
}

static int _which_fault(uint16_t motor_fault_reg, uint8_t* fault_flg)
{
    uint8_t i = 0;

    for (i = 0; i < IRMCK099_FAULT_NUM; i++) {
        fault_flg[i] = 0;
    }

    for (i = 0; i < IRMCK099_FAULT_NUM; i++) {

        if (motor_fault_reg & (0x01 << i)) {
            fault_flg[i] = 1;
        }
    }

    return FC_OK;
}

static int16_t _std_msg_sum(struct irmck099_tx_std_msg_s *tx_std_msg)
{
    return - (_tx_std_msg.node_addr << 8 | _tx_std_msg.cmd_num)
        - _tx_std_msg.data_word0
        - _tx_std_msg.data_word1;
}

static int16_t _long_msg_sum(struct irmck099_tx_long_msg_s *tx_long_msg)
{
    return - (_tx_long_msg.node_addr << 8 | _tx_long_msg.cmd_num)
        - _tx_long_msg.data_word0
        - _tx_long_msg.data_word1
        - _tx_long_msg.data_word2
        - _tx_long_msg.data_word3;
}

static int _transmit_std_msg_prepare(void)
{
    _tx_buf[0] = _tx_std_msg.node_addr;
    _tx_buf[1] = _tx_std_msg.cmd_num;
    _tx_buf[2] = BYTE1(_tx_std_msg.data_word0);
    _tx_buf[3] = BYTE0(_tx_std_msg.data_word0);
    _tx_buf[4] = BYTE1(_tx_std_msg.data_word1);
    _tx_buf[5] = BYTE0(_tx_std_msg.data_word1);
    _tx_buf[6] = BYTE1(_tx_std_msg.check_sum);
    _tx_buf[7] = BYTE0(_tx_std_msg.check_sum);

    return FC_OK;
}

static int _transmit_long_msg_prepare(void)
{
    _tx_buf[0] = _tx_long_msg.node_addr;
    _tx_buf[1] = _tx_long_msg.cmd_num;
    _tx_buf[2] = BYTE1(_tx_long_msg.data_word0);
    _tx_buf[3] = BYTE0(_tx_long_msg.data_word0);
    _tx_buf[4] = BYTE1(_tx_long_msg.data_word1);
    _tx_buf[5] = BYTE0(_tx_long_msg.data_word1);
    _tx_buf[6] = BYTE1(_tx_long_msg.data_word2);
    _tx_buf[7] = BYTE0(_tx_long_msg.data_word2);
    _tx_buf[8] = BYTE1(_tx_long_msg.data_word3);
    _tx_buf[9] = BYTE0(_tx_long_msg.data_word3);
    _tx_buf[10] = BYTE1(_tx_long_msg.check_sum);
    _tx_buf[11] = BYTE0(_tx_long_msg.check_sum);

    return FC_OK;
}

static int16_t _check_msg_sum(char *buffer, size_t num_bytes)
{
    int16_t sum = 0;
    uint8_t cnt = 0;

    //检验msg所有字相加等于0
    for (cnt = 0; cnt < (num_bytes / 2); cnt++) {
        sum += (buffer[2 * cnt] << 8 | buffer[2 * cnt + 1]);
    }

    return sum;
}


static int _receive_analysis(char *buffer, size_t num_bytes)
{
    if (num_bytes == IRMCK099_STD_MSG_LEN) {
        if (_check_msg_sum(buffer, num_bytes) != 0) {
           PX4_INFO("FOC receive the check sum std is not right\n");
            return FC_FAIL;
        }
    } else if (num_bytes != IRMCK099_LONG_MSG_LEN) {
        if (_check_msg_sum(buffer, num_bytes) == 0) {
           PX4_INFO("FOC receive the check sum !long is not right\n");
            return FC_FAIL;
        }
    }

    int _motor_index = -1, _boardcast_flg = -1;

    //判断是哪个电机传回的数据
    switch (buffer[0]) {
    case IRMCK099_BOARDCAST_ADDR:
        _boardcast_flg = 1;
        break;
    case IRMCK099_MOTOR0_ADDR:
        _motor_index = 0;
        break;
    case IRMCK099_MOTOR1_ADDR:
        _motor_index = 1;
        break;
    case IRMCK099_MOTOR2_ADDR:
        _motor_index = 2;
        break;
    case IRMCK099_MOTOR3_ADDR:
        _motor_index = 3;
        break;
    default:
        PX4_INFO("FOC receive addr error faild\r\n");
        return FC_FAIL;
    }

    //判断返回值是哪个电机的速度值
    if (_boardcast_flg == 1) {
        switch (buffer[1]) {
        case IRMCK099_RX_FLG | IRMCK099_BOARDCAST_CHANNEL0_FLAG:
            _motor_data[IRMCK099_CHANNEL0_MOTOR_INDEX].speed     = (uint16_t)((buffer[2] << 8 | buffer[3]) * IRMCK099_SPEED_RAW_TO_RPM_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL0_MOTOR_INDEX].current   = (uint16_t)((buffer[4] << 8 | buffer[5]) * IRMCK099_CURRENT_RAW_TO_MA_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL0_MOTOR_INDEX].fault_reg = (buffer[6] << 8 | buffer[7]);
            break;
        case IRMCK099_RX_FLG | IRMCK099_BOARDCAST_CHANNEL1_FLAG:
            _motor_data[IRMCK099_CHANNEL1_MOTOR_INDEX].speed     = (uint16_t)((buffer[2] << 8 | buffer[3]) * IRMCK099_SPEED_RAW_TO_RPM_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL1_MOTOR_INDEX].current   = (uint16_t)((buffer[4] << 8 | buffer[5]) * IRMCK099_CURRENT_RAW_TO_MA_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL1_MOTOR_INDEX].fault_reg = (buffer[6] << 8 | buffer[7]);
            break;
        case IRMCK099_RX_FLG | IRMCK099_BOARDCAST_CHANNEL2_FLAG:
            _motor_data[IRMCK099_CHANNEL2_MOTOR_INDEX].speed     = (uint16_t)((buffer[2] << 8 | buffer[3]) * IRMCK099_SPEED_RAW_TO_RPM_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL2_MOTOR_INDEX].current   = (uint16_t)((buffer[4] << 8 | buffer[5]) * IRMCK099_CURRENT_RAW_TO_MA_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL2_MOTOR_INDEX].fault_reg = (buffer[6] << 8 | buffer[7]);
            break;
        case IRMCK099_RX_FLG | IRMCK099_BOARDCAST_CHANNEL3_FLAG:
            _motor_data[IRMCK099_CHANNEL3_MOTOR_INDEX].speed     = (uint16_t)((buffer[2] << 8 | buffer[3]) * IRMCK099_SPEED_RAW_TO_RPM_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL3_MOTOR_INDEX].current   = (uint16_t)((buffer[4] << 8 | buffer[5]) * IRMCK099_CURRENT_RAW_TO_MA_MULRIPLE);
            _motor_data[IRMCK099_CHANNEL3_MOTOR_INDEX].fault_reg = (buffer[6] << 8 | buffer[7]);
            break;
        default:
            return FC_FAIL;
        }
    }



    switch (buffer[1]) {
    case IRMCK099_RX_FLG | IRMCK099_READ_FAULT_FLG:
        _motor_data[_motor_index].fault_reg = (buffer[4] << 8 | buffer[5]);
        break;
    case IRMCK099_RX_FLG | IRMCK099_WRITE_REG:
        // PX4_INFO("FOC receive and setup _reg_addr \r\n");
        _reg_addr_for_check = (buffer[2] << 8 | buffer[3]);
        _reg_val_for_check = (buffer[4] << 8 | buffer[5]);
        break;
    default:
        return FC_FAIL;
    }

    return FC_OK;
}

//修改irmck099芯片寄存器，发送，检查是否修改成功，三次失败后报错
static int _write_register_with_check(uint8_t addr, uint16_t reg_addr, uint16_t reg_val)
{
    int result = 0;
    uint8_t i = 0;

    for (i = 0; i < 3; i++) {
        _write_register(addr, reg_addr, reg_val);

        usleep(2000);

        if (_write_register_check(reg_addr, reg_val) == 0) {
            break;
        }
    }

    if (i == 3) {
       PX4_INFO("foc write check addr: %d register: %d failed\n", addr, reg_addr);
       result = -1;
    }

    return result;
}

static int _write_register(uint8_t addr, uint16_t reg_addr, uint16_t reg_val)
{
    _tx_std_msg.len = IRMCK099_STD_MSG_LEN;
    _tx_std_msg.node_addr = addr;
    _tx_std_msg.cmd_num = IRMCK099_WRITE_REG;

    _tx_std_msg.data_word0 = reg_addr;
    _tx_std_msg.data_word1 = reg_val;
    _tx_std_msg.check_sum = _std_msg_sum(&_tx_std_msg);
    _transmit_std_msg_prepare();

    _write_data(_tx_buf, _tx_std_msg.len);

    return FC_OK;
}

static int _write_register_check(uint16_t reg_addr, uint16_t reg_val)
{
    if ((_reg_addr_for_check == reg_addr) && (_reg_val_for_check == reg_val)) {
        _reg_addr_for_check = 0;
        _reg_val_for_check = 0;
        return FC_OK;
    } else {
        return FC_FAIL;
    }
}

static int _music_mode_entry(uint8_t addr)
{
    int result = 0;

    result |= _write_register_with_check(addr, IRMCK099_MTRCTRLSEQ, IRMCK099_MOTOR_STOP);

    result |= _write_register_with_check(addr, IRMCK099_HWCONFIG, IRMCK099_MOTORID_BY_ANI3 | IRMCK099_QFN32_OUT2 | IRMCK099_NON_INVERT_AMPLIFIER);

    result |= _write_register_with_check(addr, IRMCK099_SYSCONFIG, IRMCK099_BAUDRATE_115200 | IRMCK099_DISABLE_CATCH_SPIN);

    result |= _write_register_with_check(addr, IRMCK099_ANGLESELECT, IRMCK099_OPEN_LOOP_ANGLE);

    result |= _write_register_with_check(addr, IRMCK099_CTRLMODESELECT, IRMCK099_OPEN_LOOP_VOLTAGE_CONTROL);

    result |= _write_register_with_check(addr, IRMCK099_TCNTMIN, IRMCK099_TCNTMIN_DISABLE);

    result |= _write_register_with_check(addr, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_DISABLE);

    result |= _write_register_with_check(addr, IRMCK099_SWFAULTS_MASK, IRMCK099_FAULT_CHECK_DISABLE);

    result |= _write_register_with_check(addr, IRMCK099_VD_CMD, IRMCK099_SOUND_MUTE);

    result |= _write_register_with_check(addr, IRMCK099_ROTOR_ANGLE_ADV, IRMCK099_OPEN_LOOP_ANGLE);

    result |= _write_register_with_check(addr, IRMCK099_PWMPHASEMASK, IRMCK099_ENABLE_THREE_PHASE);

    result |= _write_register_with_check(addr, IRMCK099_ACTIVEOUTTIME, IRMCK099_SOUND_VOLUME_TUNE);

    result |= _write_register_with_check(addr, IRMCK099_TARGETSPEED, IRMCK099_ZERO_SPEED);

    result |= _write_register_with_check(addr, IRMCK099_MTRCTRLSEQ, IRMCK099_MOTOR_START);

    return result;
}

static int _music_sound(uint8_t tone, uint32_t audio_time, uint32_t pause_time)
{
    _write_register(IRMCK099_BOARDCAST_ADDR, IRMCK099_VD_CMD, IRMCK099_SOUND_VOLUME_FINE_TUNE);
    usleep(2000);

    _write_register(IRMCK099_BOARDCAST_ADDR, IRMCK099_MAXRUNTIME, tone);
    usleep(audio_time);

    _write_register(IRMCK099_BOARDCAST_ADDR, IRMCK099_VD_CMD, IRMCK099_SOUND_MUTE);
    usleep(pause_time);

    return FC_OK;
}

static int _music_mode_exit(uint8_t addr)
{
    int result = 0;

    result |= _write_register_with_check(addr, IRMCK099_MTRCTRLSEQ, IRMCK099_MOTOR_STOP);

    result |= _write_register_with_check(addr, IRMCK099_HWCONFIG, IRMCK099_MOTORID_BY_ANI3 | IRMCK099_QFN32_OUT2 | IRMCK099_NON_INVERT_AMPLIFIER);

    result |= _write_register_with_check(addr, IRMCK099_SYSCONFIG, IRMCK099_BAUDRATE_115200 | IRMCK099_DISABLE_CATCH_SPIN | IRMCK099_ENABLE_DC_BUS_COMP);

    result |= _write_register_with_check(addr, IRMCK099_ANGLESELECT, IRMCK099_FLUX_ANGLE);

    result |= _write_register_with_check(addr, IRMCK099_CTRLMODESELECT, IRMCK099_CLOSE_LOOP_SPEED_CONTROL);

    result |= _write_register_with_check(addr, IRMCK099_TCNTMIN, IRMCK099_TCNTMIN_VAL);

    result |= _write_register_with_check(addr, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_VAL);

    result |= _write_register_with_check(addr, IRMCK099_SWFAULTS_MASK, IRMCK099_DC_OVER_VOLTAGE | IRMCK099_DC_UNDER_VOLTAGE | IRMCK099_OVER_TEMPERATURE | IRMCK099_ROTOR_LOCK);

    return result;
}

static int _modify_parameter(void)
{
    int result = 0;

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_REGENLIM, IRMCK099_REGENLIM_DISABLE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_REGENLIM, IRMCK099_REGENLIM_DISABLE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_REGENLIM, IRMCK099_REGENLIM_DISABLE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_REGENLIM, IRMCK099_REGENLIM_DISABLE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_SPDRAMPRATE, IRMCK099_ACCEL_LIM_150000);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_SPDRAMPRATE, IRMCK099_ACCEL_LIM_150000);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_SPDRAMPRATE, IRMCK099_ACCEL_LIM_150000);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_SPDRAMPRATE, IRMCK099_ACCEL_LIM_150000);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_KP_SPD, IRMCK099_KP_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_KP_SPD, IRMCK099_KP_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_KP_SPD, IRMCK099_KP_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_KP_SPD, IRMCK099_KP_SPD_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_KI_SPD, IRMCK099_KI_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_KI_SPD, IRMCK099_KI_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_KI_SPD, IRMCK099_KI_SPD_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_KI_SPD, IRMCK099_KI_SPD_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_DEADTIME, IRMCK099_DEADTIME_10NS);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_DEADTIME, IRMCK099_DEADTIME_10NS);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_DEADTIME, IRMCK099_DEADTIME_10NS);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_DEADTIME, IRMCK099_DEADTIME_10NS);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_FLUXTAU, IRMCK099_FLUXTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_FLUXTAU, IRMCK099_FLUXTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_FLUXTAU, IRMCK099_FLUXTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_FLUXTAU, IRMCK099_FLUXTAU_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_ATANTAU, IRMCK099_ATANTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_ATANTAU, IRMCK099_ATANTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_ATANTAU, IRMCK099_ATANTAU_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_ATANTAU, IRMCK099_ATANTAU_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_LOWSPEEDLIM, IRMCK099_LOWSPEEDLIM_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_LOWSPEEDLIM, IRMCK099_LOWSPEEDLIM_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_LOWSPEEDLIM, IRMCK099_LOWSPEEDLIM_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_LOWSPEEDLIM, IRMCK099_LOWSPEEDLIM_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_RS, IRMCK099_RS_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_RS, IRMCK099_RS_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_RS, IRMCK099_RS_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_RS, IRMCK099_RS_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_SHDELAY, IRMCK099_SHDELAY_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_SHDELAY, IRMCK099_SHDELAY_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_SHDELAY, IRMCK099_SHDELAY_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_SHDELAY, IRMCK099_SHDELAY_VALUE);

    result |= _write_register_with_check(IRMCK099_MOTOR0_ADDR, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR1_ADDR, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR2_ADDR, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_VALUE);
    result |= _write_register_with_check(IRMCK099_MOTOR3_ADDR, IRMCK099_TMINPHASESHIFT, IRMCK099_TMINPHASESHIFT_VALUE);

    return result;
}

#endif  // DRIVER_MOTOR_SELECT_IRMCK099
