#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <px4_log.h>
#include "gpio_enable_motor.h"

int enableMotorGPIO(int pin){

    //export GPIO
    char buffer[64];
    int len;
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd < 0) {
        PX4_INFO("MOTOR_GPIO open 'export' failed");
        return(-1);
    }

    len = snprintf(buffer, sizeof(buffer), "%d", pin);

    if (write(export_fd, buffer, len) < 0) {
        PX4_INFO("MOTOR_GPIO motor already enable.");
        close(export_fd);
        return 0;
    }

    PX4_INFO("MOTOR_GPIO %d Export success",pin);
    close(export_fd);


    //Setup GPIO direction
    static const char dir_str[] = "in\0out";
    int dir = 1 ; //dir: 0-->IN, 1-->OUT
    char directionPath[128];
    snprintf(directionPath, sizeof(directionPath), "/sys/class/gpio/gpio%d/direction", pin);
    int direction_fd = open(directionPath, O_WRONLY);
    if (direction_fd < 0) {
        PX4_INFO("MOTOR_GPIO open 'direction' failed");
        return -2;
    }
    if (write(direction_fd, &dir_str[dir == 0 ? 0 : 3], dir == 0 ? 2 : 3) < 0) {
        PX4_INFO("MOTOR_GPIO setup 'direction' failed");
        return -3;
    }
    PX4_INFO("MOTOR_GPIO setup 'direction' out success");
    close(direction_fd);

    //Setup GPIO value
    static const char values_str[] = "01";
    int value = 1; //value: 0-->LOW, 1-->HIG
    char valut_path[128];
    snprintf(valut_path, sizeof(valut_path), "/sys/class/gpio/gpio%d/value", pin);
    int value_fd = open(valut_path, O_WRONLY);
    if (value_fd < 0) {
        PX4_INFO("MOTOR_GPIO open 'value' failed");
        return -3;
    }
    if (write(value_fd, &values_str[value == 0 ? 0 : 1], 1) < 0) {
        PX4_INFO("MOTOR_GPIO setup 'value' failed");
        return -4;
    }
    PX4_INFO("MOTOR_GPIO setip 'value' %d success: %d", value);
    PX4_INFO("MOTOR_GPIO enable motor success");

    close(value_fd);
    return 0;
    
}
