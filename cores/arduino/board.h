#ifndef _BOARD_H_
#define _BOARD_H_


#include "interrupt.h"
#include "analog.h"
// #include "backup.h"
#include "clock.h"
// #include "core_callback.h"
#include "digital_io.h"
// #include "dwt.h"
#include "hw_config.h"
// #include "otp.h"
#include "timer.h"
#include "uart.h"


/*
 * Core and peripherals registers definitions
*/
#ifdef __cplusplus
extern "C" {
#endif

void pre_init(void) ;

#ifdef __cplusplus
}
#endif


#endif /* _BOARD_H_ */
