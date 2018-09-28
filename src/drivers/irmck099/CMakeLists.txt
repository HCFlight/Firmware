px4_add_module(
        MODULE drivers__irmck099
        MAIN irmck099
        COMPILE_FLAGS
        SRCS
        irmck099.cpp
         drv_motor.c drv_irmck099.c
        DEPENDS
        mixer
        pwm_limit
)