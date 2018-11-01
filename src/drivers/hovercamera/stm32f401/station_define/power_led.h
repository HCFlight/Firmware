#ifndef _FC_STATION__INC__DEFINES__F401__POWER_LED_H_
#define _FC_STATION__INC__DEFINES__F401__POWER_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

enum f401_breathled_mode_e {
    F401_BREATHLED_MODE_OFF = 0,
    F401_BREATHLED_MODE_ON,
    F401_BREATHLED_MODE_BLINK,
    F401_BREATHLED_MODE_BREATH
};

#ifdef __cplusplus
}
#endif

#endif
