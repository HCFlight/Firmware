

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <cmath>	// NAN

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <mixer/mixer.h>
#include <mixer/mixer_multirotor.generated.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <pwm_limit/pwm_limit.h>
#include <dev_fs_lib_serial.h>

extern "C"
{

#include "drv_motor.h"
#include "drv_irmck099.h"
}
#define MAXMOTORSPEED 18000
namespace irmck099
{

volatile bool _task_should_exit = false;
static bool _is_running = false;
static px4_task_t _task_handle = -1;
//adsp
static char _mixer_filename[32] = "/dev/fs/hc1.main.mix";
MultirotorMixer *_mixer = nullptr;

// subscriptions
int _controls_sub;
int _armed_sub;
int _fd;

orb_advert_t _outputs_pub = nullptr;
// topic structures
actuator_controls_s _controls;
actuator_outputs_s _outputs;
actuator_armed_s _armed;

// polling
uint8_t _poll_fds_num = 0;
px4_pollfd_struct_t _poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

// limit for pwm
pwm_limit_t _pwm_limit;

// esc parameters
int32_t _pwm_disarmed = 1000;
int32_t _pwm_min = 1050;
int32_t _pwm_max = 2000;

//perf_counter_t	_perf_control_latency = nullptr;

int start();
void stop();
void usage();
void task_main_trampoline(int argc, char *argv[]);
void task_main(int argc, char *argv[]);
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);
void pwm_to_speed(uint16_t pwm[], uint16_t motorspeed[]);

void pwm_to_speed(uint16_t pwm[], int16_t motorspeed[])
{
    float speed = 0;
    speed = MAXMOTORSPEED / (2000 - 1000);
    motorspeed[1] = speed * (pwm[0] - 1000);
    motorspeed[3] = speed * (pwm[1] - 1000);
    motorspeed[0] = speed * (pwm[2] - 1000);
    motorspeed[2] = speed * (pwm[3] - 1000);
}

int mixer_control_callback(uintptr_t handle,
                           uint8_t control_group,
                           uint8_t control_index,
                           float &input)
{
    const actuator_controls_s *controls = (actuator_controls_s *)handle;

    input = controls[control_group].control[control_index];

    return 0;
}
int initialize_mixer(const char *mixer_filename)
{

    char buf[2048];
    size_t buflen = sizeof(buf);
    PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);
    int fd_load = ::open(mixer_filename, O_RDONLY);

    if (fd_load != -1)
    {
        int nRead = ::read(fd_load, buf, buflen);
        close(fd_load);

        if (nRead > 0)
        {
            _mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

            if (_mixer != nullptr)
            {
                PX4_INFO("Successfully initialized mixer from config file");
                return 0;
            }
            else
            {
                PX4_ERR("Unable to parse from mixer config file");
                return -1;
            }
        }
        else
        {
            PX4_WARN("Unable to read from mixer config file");
            return -2;
        }
    }
    else
    {
        PX4_WARN("No mixer config file found, using default mixer.");

        /* Mixer file loading failed, fall back to default mixer configuration for
            * QUAD_X airframe. */
        float roll_scale = 1;
        float pitch_scale = 1;
        float yaw_scale = 1;
        float deadband = 0;

        _mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
                                     MultirotorGeometry::QUAD_H,
                                     roll_scale, pitch_scale, yaw_scale, deadband);

        if (_mixer == nullptr)
        {
            return -1;
        }

        return 0;
    }
}
void task_main(int argc, char *argv[])
{

    PX4_INFO("irmck099_: task main");

    int initRslt = drv_motor_init();

    PX4_INFO("irmck099_: motor_init result :%d", initRslt);

    int playRslt = irmck099_play_music(DYNAMIC_MOTOR_MUSIC_INDEX_POWER_ON);

    PX4_INFO("irmck099_: play result :%d", playRslt);

    if (initialize_mixer(_mixer_filename) < 0)
    {
        PX4_ERR("Mixer initialization failed.");
        return;
    }
    // Subscribe for orb topics
    _controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));

    // Start disarmed
    _armed.armed = false;
    _armed.prearmed = false;
    // Set up poll topic
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _controls_sub;
    fds[0].events = POLLIN;

    pwm_limit_init(&_pwm_limit);

    while (!_task_should_exit)
    {

        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);
        /* This is undesirable but not much we can do. */
        if (pret < 0)
        {
            PX4_WARN("poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }
        if (fds[0].revents & POLLIN)
        {
            orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);
            _outputs.timestamp = _controls.timestamp;

            /* do mixing */
            _outputs.noutputs = _mixer->mix(_outputs.output, 4);

            /* disable unused ports by setting their output to NaN */
            for (size_t i = _outputs.noutputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++)
            {
                _outputs.output[i] = NAN;
            }

            const uint16_t reverse_mask = 0;
            uint16_t disarmed_pwm[4];
            uint16_t min_pwm[4];
            uint16_t max_pwm[4];

            for (unsigned int i = 0; i < 4; i++)
            {
                disarmed_pwm[i] = _pwm_disarmed;
                min_pwm[i] = _pwm_min;
                max_pwm[i] = _pwm_max;
            }

            uint16_t pwm[4];
            int16_t motorspeed[4];
            // TODO FIXME: pre-armed seems broken
            pwm_limit_calc(_armed.armed, false /*_armed.prearmed*/, _outputs.noutputs, reverse_mask,
                           disarmed_pwm, min_pwm, max_pwm, _outputs.output, pwm, &_pwm_limit);
            pwm_to_speed(pwm, motorspeed);
            irmck099_set_speed_four_motor(motorspeed);
            if (_outputs_pub != nullptr)
            {
                orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);
            }
            else
            {
                _outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
            }
            bool updated;
            orb_check(_armed_sub, &updated);

            if (updated)
            {
                orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
            }
        }
    }
    orb_unsubscribe(_controls_sub);
    orb_unsubscribe(_armed_sub);

    _is_running = false;
}

void task_main_trampoline(int argc, char *argv[])
{
    PX4_INFO("irmck099_: task_main_trampoline");
    task_main(argc, argv);
}

int start()
{
    PX4_INFO("irmck099 start");
    if (_is_running)
    {
        PX4_INFO("irmck099 _is_running");
        return -1;
    }
    ASSERT(_task_handle == -1);
    _task_should_exit = false;
    _task_handle = px4_task_spawn_cmd("irmck099_main",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      2000,
                                      (px4_main_t)&task_main_trampoline,
                                      nullptr);
    if (_task_handle < 0)
    {
        PX4_ERR("task start failed");
        return -1;
    }
    return 0;
}

void stop()
{
    PX4_INFO("irmck099 stop");
}

void usage()
{
    PX4_INFO("Usage: irmck099 {start|info|stop}");
}

void test()
{
    PX4_INFO("irmck099 test");
}

} // namespace irmck099

extern "C" __EXPORT int irmck099_main(int argc, char *argv[]);

int irmck099_main(int argc, char *argv[])
{

    return irmck099::start();

    PX4_INFO("irmck099_debug: irmck099_main :%d cmp:%d\r", strlen(argv[1]), strncmp(argv[1], "test", 4));
    if (strncmp(argv[1], "start", 5))
    {
        return irmck099::start();
    }
    else if (!strcmp(argv[1], "stop"))
    {
        irmck099::stop();
        return 1;
    }
    else if (strncmp(argv[1], "test", 4) == 0)
    {

        irmck099::test();
        return 1;
    }
    else
    {
        irmck099::usage();
        return 1;
    }
}