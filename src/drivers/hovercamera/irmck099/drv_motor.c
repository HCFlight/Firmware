#include "drv_motor.h"

#include "fc_define/motor.h"

#if defined DRIVER_MOTOR_SELECT_C8051

#include "c8051/drv_c8051.h"

#elif defined DRIVER_MOTOR_SELECT_IRMCK099

#include "drv_irmck099.h"

#endif

int drv_motor_init(void)
{
#if defined DRIVER_MOTOR_SELECT_C8051

    return c8051_init(C8051_SPI_PATH);

#elif defined DRIVER_MOTOR_SELECT_IRMCK099

    return irmck099_motor_init();

#else

    return FC_FAIL;

#endif
}

int drv_motor_update(const struct dynamic_motor_output_s *motor_output)
{
#if defined DRIVER_MOTOR_SELECT_C8051

    /*c8051_set_motor_speed((uint8_t)motor_output->set_speed[0], 0);
    c8051_set_motor_speed((uint8_t)motor_output->set_speed[1], 1);
    c8051_set_motor_speed((uint8_t)motor_output->set_speed[2], 2);
    c8051_set_motor_speed((uint8_t)motor_output->set_speed[3], 3);*/

#elif defined DRIVER_MOTOR_SELECT_IRMCK099

    int16_t _motor_output[4] = {
        (int16_t)motor_output->set_speed[0],
        (int16_t)motor_output->set_speed[1],
        (int16_t)motor_output->set_speed[2],
        (int16_t)motor_output->set_speed[3]
    };
    return irmck099_set_speed_four_motor(_motor_output);

#else

    return FC_FAIL;

#endif
}

int drv_motor_get_data(struct dynamic_motor_raw_s *motor_data)
{
#if defined DRIVER_MOTOR_SELECT_C8051

    int16_t _motor_speeds[4];
    static int index = 0;
    c8051_get_motor_speed(&_motor_speeds[index], index);
    if (++index < DYNAMIC_MOTOR_NUM)
        return FC_FAIL;
    motor_data->speed[0] = _motor_speeds[0];
    motor_data->speed[1] = _motor_speeds[1];
    motor_data->speed[2] = _motor_speeds[2];
    motor_data->speed[3] = _motor_speeds[3];
    return FC_OK;

#elif defined DRIVER_MOTOR_SELECT_IRMCK099

    int16_t _motor_speeds[4];
    static uint8_t index = 0;
    if (++index < DYNAMIC_MOTOR_NUM)
        return FC_FAIL;
    index = 0;
    irmck099_get_speed(_motor_speeds);
    motor_data->speed[0] = _motor_speeds[0];
    motor_data->speed[1] = _motor_speeds[1];
    motor_data->speed[2] = _motor_speeds[2];
    motor_data->speed[3] = _motor_speeds[3];
    return FC_OK;

#else

    return FC_FAIL;

#endif
}
