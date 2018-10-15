#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <string.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

extern "C" {
#include "gpio_enable_motor.h"
}

namespace hc1_enable_motor{

    volatile bool _task_should_exit = false;
    static bool _is_running = false;
    static px4_task_t _task_handle = -1;

    int start();
    void stop();
    void usage();
    void task_main_trampoline(int argc, char *argv[]);
    void task_main(int argc, char *argv[]);
  
  
    void task_main(int argc, char *argv[])
    {
        enableMotorGPIO(HC1_MOTOR_GPIO_NUM);
    }

    void task_main_trampoline(int argc, char *argv[])
    {
        task_main(argc,argv);
    }

    int start()
    {
       if(_is_running){
            PX4_INFO("hc1_enable_motor _is_running");
            return -1;
        }
        ASSERT(_task_handle == -1);
        _task_should_exit = false;
        _task_handle = px4_task_spawn_cmd("hc1_enable_motor_main",
                                          SCHED_DEFAULT,
                                          SCHED_PRIORITY_DEFAULT,
                                          1500,
                                          (px4_main_t)&task_main_trampoline,
                                          nullptr);
        if (_task_handle < 0) {
            PX4_ERR("task start failed");
            return -1;
        }
        return 0;
    }

}

extern "C" __EXPORT int hc1_enable_motor_main(int argc, char *argv[]);

int hc1_enable_motor_main(int argc, char *argv[])
{
    return hc1_enable_motor::start();
}