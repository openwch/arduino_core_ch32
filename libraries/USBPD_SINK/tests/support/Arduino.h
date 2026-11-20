#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#include <stdint.h>

#if defined(TIM_MODULE_ENABLED)
#define TIM3 3
#define MICROSEC_FORMAT 0

class HardwareTimer
{
public:
    explicit HardwareTimer(int);
    void setOverflow(uint32_t, int);
    void attachInterrupt(void (*)());
    void detachInterrupt();
    void pause();
    void resume();
};
#endif

#endif
