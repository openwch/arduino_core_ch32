/*
 * AVR <util/delay.h> compatibility for libraries written for AVR
 * (e.g. Adafruit_SSD1306 includes it on unknown platforms).
 * Maps the busy-wait helpers onto the Arduino core timing functions.
 */
#ifndef _UTIL_DELAY_H_
#define _UTIL_DELAY_H_

#include "wiring_time.h"

#ifndef _delay_ms
#define _delay_ms(ms)  delay((uint32_t)(ms))
#endif

#ifndef _delay_us
#define _delay_us(us)  delayMicroseconds((uint32_t)(us))
#endif

#endif /* _UTIL_DELAY_H_ */
