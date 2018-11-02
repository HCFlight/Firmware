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

#include "drv_f401.h"
   
}



namespace stm32f401{

    volatile bool _task_should_exit = false;
    static bool _is_running = false;
    static px4_task_t _task_handle = -1;
    orb_advert_t distance_sensor_pub;
    int start();
    void stop();
    void usage();
    void task_main_trampoline(int argc, char *argv[]);
    void task_main(int argc, char *argv[]);
  
  
    void task_main(int argc, char *argv[])
    {

        int initResult =  f401_init("/dev/spi-10");
        if (initResult == F401_ERROR_CODE_FAIL) {
            PX4_ERR("HC_ERROR stm32f401_: init failed result: %d", initResult);
        } else {
            PX4_INFO("stm32f401_: init success result: %d", initResult);
        }

        int _orb_class_instance = -1;

        // //len freq setup test F401 driver
        // int ledsetup = f401_power_led_set_freq(10);
        // PX4_INFO("stm32f401_: led setup result: %d", ledsetup);
        // if (ledsetup == F401_ERROR_CODE_FAIL) {
        //     PX4_INFO("stm32f401_: led setup failed result: %d", ledsetup);
        //     return;
        // }

        usleep(1000 * 100);

        int jumpToAppResult = f401_jump_to_app();
        if (jumpToAppResult == F401_ERROR_CODE_FAIL) {
            PX4_ERR("HC_ERROR stm32f401_: jump to app falied result: %d", jumpToAppResult);
            return;
        } else {
            PX4_INFO("stm32f401_: jump to app success result: %d", jumpToAppResult);
        }

        usleep(1000 * 100);

        int pulseSetupResult = f401_sonar_set_pulse(0, 10);
        if (pulseSetupResult == F401_ERROR_CODE_FAIL) {
            PX4_ERR("HC_ERROR stm32f401_: sonar pulse setup failed result: %d", pulseSetupResult);
            return;
        } else {
            PX4_INFO("stm32f401_: sonar pulse setup success result: %d", pulseSetupResult);
        }
    
      
        //Start sonar and setup.
        int sonarStartResult = f401_sonar_start();
        if (sonarStartResult == F401_ERROR_CODE_FAIL) {
            PX4_ERR("HC_ERROR stm32f401_: start sonar failed result: %d", sonarStartResult);
            return;
        } else {
            PX4_INFO("stm32f401_: start sonar result: %d", sonarStartResult);
        }

      
        //Mock publish data 
        struct distance_sensor_s report;
        uint32_t sonarHegiht = 0;
        struct sensor_sonar_raw_s sonarRaw; 

        while(!_task_should_exit)
        {

            int getHeightResult = f401_sonar_get_height(&sonarHegiht);
            if (getHeightResult == F401_ERROR_CODE_UNINITIATED) {
                 PX4_INFO("stm32f401_: sonar get height failed result: %d", getHeightResult);
            } 

            int getRawResult = f401_sonar_get_raw(&sonarRaw); 
            if (getRawResult == F401_ERROR_CODE_UNINITIATED) {
                 PX4_INFO("stm32f401_: sonar get height failed result: %d", getRawResult);
            } 

        report.timestamp = hrt_absolute_time();
        report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
        report.orientation = 25;
        report.current_distance = float(sonarHegiht)/1000;
        report.min_distance = 0.4;
        report.max_distance = 5.1;
        report.covariance = 0.0f;
        report._padding0[0] = 1;
        report._padding0[1] = 1;
        report._padding0[2] = 1;
        report._padding0[3] = 1;
        report._padding0[4] = 1;
        /* TODO: set proper ID */
        report.id = 0;
            if(distance_sensor_pub != nullptr)
            {
                //PX4_INFO("stm32f401_: publish height : %d", sonarHegiht);
                orb_publish(ORB_ID(distance_sensor), distance_sensor_pub, &report);

            }else{
                 distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &report,&_orb_class_instance, ORB_PRIO_LOW);
            }

        }
       orb_unadvertise(distance_sensor_pub);
    }

    void task_main_trampoline(int argc, char *argv[])
    {
        task_main(argc,argv);
    }

    int start()
    {
       if(_is_running){
            PX4_INFO("stm32f401 _is_running");
            return -1;
        }
        ASSERT(_task_handle == -1);
        _task_should_exit = false;
        _task_handle = px4_task_spawn_cmd("stm32f401_main",
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

    void stop()
    {
        PX4_INFO("stm32f401 stop");
    }

    void usage()
    {
        PX4_INFO("Usage: stm32f401 {start|info|stop}");
    }

    void test()
    {
        PX4_INFO("stm32f401 test");
    }
}

extern "C" __EXPORT int stm32f401_main(int argc, char *argv[]);

int
stm32f401_main(int argc, char *argv[])
{
    return stm32f401::start();

    if(strncmp(argv[1],"start", 5)==0)
    {
        return stm32f401::start();
    } else if(strncmp(argv[1],"stop", 5)==0)
    {
         stm32f401::stop();
        return 1;
    } else if(strncmp(argv[1],"test",4 ) == 0) {
    
         stm32f401::test();
        return 1;
    } else
    {
        stm32f401::usage();
        return 1;
    }
}