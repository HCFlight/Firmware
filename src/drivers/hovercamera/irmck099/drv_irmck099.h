/************************************************************************
    @file: drv_motor_foc.h
    @Author: huang Pengfei
    @Mail: huangpengfei@zerozero.cn
    @Created Time: 2016/5/3 15:52:11
    @copyright Copyright (c) 2014-2015 Beijing Zero Zero Infinity Technology Co., Ltd all right reserved.
************************************************************************/
#ifndef __DRIVER_IRMCK099_H_
#define __DRIVER_IRMCK099_H_

#include "fc_define/motor.h"

#define DRIVER_MOTOR_SELECT_IRMCK099 1

#ifdef DRIVER_MOTOR_SELECT_IRMCK099

#include <dev_fs_lib_serial.h>

#include "fc_define/basic_type.h"

#define IRMCK099_UART_PATH "/dev/tty-4"

#define IRMCK099_CFG_FIELDS \
    .irmck099_uart_path = IRMCK099_UART_PATH, \
    .irmck099_uart_freq = DSPAL_SIO_BITRATE_115200, \

#define IRMCK099_MOTOR_NUM                      4    // 由硬件决定，勿改

#define IRMCK099_MODE_STANDBY                   0x00
#define IRMCK099_MODE_PLAY_MUSIC                0x01
#define IRMCK099_MODE_MOTOR_RUN                 0x02

#define IRMCK099_STD_MSG_LEN                    0x08
#define IRMCK099_LONG_MSG_LEN                   0x0C

#define IRMCK099_RX_FLG                         0x80

#define IRMCK099_BOARDCAST_ADDR                 0x00
#define IRMCK099_MOTOR0_ADDR                    0x02
#define IRMCK099_MOTOR1_ADDR                    0x01
#define IRMCK099_MOTOR2_ADDR                    0x04
#define IRMCK099_MOTOR3_ADDR                    0x03
#define IRMCK099_CHANNEL0_MOTOR_INDEX           1
#define IRMCK099_CHANNEL1_MOTOR_INDEX           0
#define IRMCK099_CHANNEL2_MOTOR_INDEX           3
#define IRMCK099_CHANNEL3_MOTOR_INDEX           2

#define IRMCK099_READ_STATUS                    0x00
#define IRMCK099_SET_ONE_SPEED                  0x03
#define IRMCK099_WRITE_REG                      0x06
#define IRMCK099_BOARDCAST_CHANNEL0_FLAG        0x08
#define IRMCK099_BOARDCAST_CHANNEL1_FLAG        0x09
#define IRMCK099_BOARDCAST_CHANNEL2_FLAG        0x0A
#define IRMCK099_BOARDCAST_CHANNEL3_FLAG        0x0B

#define IRMCK099_READ_FAULT_FLG                 0x0000
#define IRMCK099_READ_NODE_ADDR                 0x0003

#define IRMCK099_FAULT_NUM                      0x0D
#define IRMCK099_MOTOR_GATEKILL_INDEX           0x00
#define IRMCK099_CRITICAL_OVER_VOLTAGE_INDEX    0x01
#define IRMCK099_OVER_VOLTAGE_INDEX             0x02
#define IRMCK099_UNDER_VOLTAGE_INDEX            0x03
#define IRMCK099_FLUX_PLL_FAULT_INDEX           0x04
#define IRMCK099_ZERO_SPEED_INDEX               0x05
#define IRMCK099_OVER_TEMPERATURE_INDEX         0x06
#define IRMCK099_ROTOR_LOCK_INDEX               0x07
#define IRMCK099_FAULT_UNUSED0                  0x08
#define IRMCK099_FAULT_UNUSED1                  0x09
#define IRMCK099_MCE_EXECUTION_FAULT_INDEX      0x0A
#define IRMCK099_FAULT_UNUSED2                  0x0B
#define IRMCK099_PARAMETER_LOAD_FAULT_INDEX     0x0C
#define IRMCK099_LINK_BREAK_INDEX               0x0D

#define IRMCK099_HWCONFIG                       0x00BD
#define IRMCK099_SYSCONFIG                      0x00BE
#define IRMCK099_ANGLESELECT                    0x01DE
#define IRMCK099_CTRLMODESELECT                 0x01F8
#define IRMCK099_TCNTMIN                        0x010D
#define IRMCK099_TMINPHASESHIFT                 0x0020
#define IRMCK099_SWFAULTS_MASK                  0x00C3
#define IRMCK099_VD_CMD                         0x01F9
#define IRMCK099_ROTOR_ANGLE_ADV                0x009B
#define IRMCK099_PWMPHASEMASK                   0x00BC
#define IRMCK099_ACTIVEOUTTIME                  0x0160
#define IRMCK099_MAXRUNTIME                     0x010B
#define IRMCK099_TARGETSPEED                    0x015E
#define IRMCK099_MTRCTRLSEQ                     0x013D
#define IRMCK099_REGENLIM                       0x01CF
#define IRMCK099_SPDRAMPRATE                    0x016C
#define IRMCK099_KP_SPD                         0x01C6
#define IRMCK099_KI_SPD                         0x01C8
#define IRMCK099_DEADTIME                       0x0E87
#define IRMCK099_FLUXTAU                        0x006C
#define IRMCK099_ATANTAU                        0x00A5
#define IRMCK099_LOWSPEEDLIM                    0x01CD
#define IRMCK099_RS                             0x0073
#define IRMCK099_SHDELAY                        0x0024
#define IRMCK099_TMINPHASESHIFT                 0x0020

#define IRMCK099_MOTORID_BY_ANI3                0x0200
#define IRMCK099_QFN32_OUT2                     0x0020
#define IRMCK099_NON_INVERT_AMPLIFIER           0x0002
#define IRMCK099_BAUDRATE_115200                0x2000
#define IRMCK099_DISABLE_CATCH_SPIN             0x0080
#define IRMCK099_ENABLE_DC_BUS_COMP             0x0010
#define IRMCK099_OPEN_LOOP_ANGLE                0x0000
#define IRMCK099_FLUX_ANGLE                     0x0002
#define IRMCK099_OPEN_LOOP_VOLTAGE_CONTROL      0x0000
#define IRMCK099_CLOSE_LOOP_SPEED_CONTROL       0x0002
#define IRMCK099_TCNTMIN_DISABLE                0x0000
#define IRMCK099_TCNTMIN_VAL                    0x03D7
#define IRMCK099_TMINPHASESHIFT_DISABLE         0x0000
#define IRMCK099_TMINPHASESHIFT_VAL             0x07AE
#define IRMCK099_FAULT_CHECK_DISABLE            0x0000
#define IRMCK099_DC_OVER_VOLTAGE                0x0004
#define IRMCK099_DC_UNDER_VOLTAGE               0x0008
#define IRMCK099_OVER_TEMPERATURE               0x0040
#define IRMCK099_ROTOR_LOCK                     0x0080
#define IRMCK099_OPEN_LOOP_ANGEL                0x0000
#define IRMCK099_ENABLE_THREE_PHASE             0x0FFF
#define IRMCK099_ZERO_SPEED                     0x0000
#define IRMCK099_MOTOR_STOP                     0x0000
#define IRMCK099_MOTOR_START                    0x0002
#define IRMCK099_REGENLIM_DISABLE               0x0000
#define IRMCK099_REGENLIM_50_PCT                0x2048
#define IRMCK099_ACCEL_LIM_150000               6828
#define IRMCK099_KP_SPD_VALUE                   300
#define IRMCK099_KI_SPD_VALUE                   20
#define IRMCK099_DEADTIME_10NS                  10
#define IRMCK099_FLUXTAU_VALUE                  7802
#define IRMCK099_ATANTAU_VALUE                  230
#define IRMCK099_LOWSPEEDLIM_VALUE              20121
#define IRMCK099_RS_VALUE                       8159
#define IRMCK099_SHDELAY_VALUE                  64500
#define IRMCK099_TMINPHASESHIFT_VALUE           2000

#define IRMCK099_DO                             57
#define IRMCK099_RA                             51
#define IRMCK099_MI                             45
#define IRMCK099_FA                             43
#define IRMCK099_SO                             38
#define IRMCK099_LA                             34
#define IRMCK099_SI                             30

#define IRMCK099_SOUND_VOLUME_TUNE              2
#define IRMCK099_SOUND_VOLUME_FINE_TUNE         4900
#define IRMCK099_SOUND_MUTE                     0

#define IRMCK099_SOUND_LONG                     400000
#define IRMCK099_SOUND_STD                      80000
#define IRMCK099_SOUND_SHORT                    200000
#define IRMCK099_PAUSE_LONG                     100000
#define IRMCK099_PAUSE_STD                      80000
#define IRMCK099_PAUSE_SHORT                    25000

#define IRMCK099_SPEED_RAW_TO_RPM_MULRIPLE      1.5f
#define IRMCK099_SPEED_RPM_TO_RAW_MULRIPLE      0.6666666667f
#define IRMCK099_CURRENT_RAW_FULL_RANGE         4096
#define IRMCK099_CURRENT_MA_FULL_RANGE          6000
#define IRMCK099_CURRENT_RAW_TO_MA_MULRIPLE     ((float)IRMCK099_CURRENT_MA_FULL_RANGE / IRMCK099_CURRENT_RAW_FULL_RANGE)

/*************************************************************************
串口帧协议：
标准帧：irmck099_tx_std_msg_s，8个字节
| 节点地址（一字节）|
| 命令号  （一字节）|
| 数据字0 （高字节）|
| 数据字0 （低字节）|
| 数据字1 （高字节）|
| 数据字1 （低字节）|
| 校验和  （高字节）|
| 校验和  （低字节）|
************************************************************************/
struct irmck099_tx_std_msg_s {
    uint8_t  node_addr;
    uint8_t  cmd_num;
    uint16_t data_word0;
    uint16_t data_word1;
    int16_t  check_sum;
    uint8_t  len;
};

/*************************************************************************
长帧：irmck099_tx_long_msg_s, 12个字节
| 节点地址（一字节）|
| 命令号  （一字节）|
| 数据字0 （高字节）|
| 数据字0 （低字节）|
| 数据字1 （高字节）|
| 数据字1 （低字节）|
| 数据字2 （高字节）|
| 数据字2 （低字节）|
| 数据字3 （高字节）|
| 数据字3 （低字节）|
| 校验和  （高字节）|
| 校验和  （低字节）|
************************************************************************/
struct irmck099_tx_long_msg_s {
    uint8_t  node_addr;
    uint8_t  cmd_num;
    uint16_t data_word0;
    uint16_t data_word1;
    uint16_t data_word2;
    uint16_t data_word3;
    int16_t  check_sum;
    uint8_t  len;
};

/*************************************************************************
校验和方法：
[节点地址|命令号]+数据字0+数据字1+(数据字2+数据字3)+校验和 = 0
************************************************************************/

struct irmck099_config_s {
    const char *irmck099_uart_path;
    struct dspal_serial_ioctl_data_rate irmck099_uart_freq;
    struct dspal_serial_ioctl_receive_data_callback receive_callback;
};

struct irmck099_motor_data_s {
    uint16_t speed;     // 转速，rpm
    uint16_t current;   // 电流，mA
    uint16_t fault_reg; // 错误标志位寄存器值
};

int irmck099_motor_init(void);

int irmck099_motor_deinit(void);

int irmck099_set_speed_four_motor(int16_t motor_speed[IRMCK099_MOTOR_NUM]);

int irmck099_get_motor_data(struct irmck099_motor_data_s *motor[IRMCK099_MOTOR_NUM]);

int irmck099_get_speed(int16_t cur_motor_speed[IRMCK099_MOTOR_NUM]);

int irmck099_get_current(uint16_t cur_motor_current[IRMCK099_MOTOR_NUM]);

int irmck099_get_fault(uint16_t cur_motor_fault[IRMCK099_MOTOR_NUM]);

int irmck099_play_music(uint8_t index);

#endif  // DRIVER_MOTOR_SELECT_IRMCK099

#endif
