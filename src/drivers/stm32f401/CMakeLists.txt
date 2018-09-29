px4_add_module(
        MODULE drivers__stm32f401
        MAIN stm32f401
        COMPILE_FLAGS
        -I ../src/lib/DriverFramework/dspal/include
        SRCS
        stm32f401.cpp drv_f401_static.c drv_f401.c compare_and_swap.c
        #stm32f401.cpp drv_f401_static.c
        DEPENDS
        platforms__common
)