#ifndef _CH32_DEF_BUILD_
#define _CH32_DEF_BUILD_

#if !defined(COM_STARTUP_FILE)
  #if defined(CH32V00x)
     #define  COM_STARTUP_FILE "startup_ch32v00x.S"
  #elif defined(CH32V203)
    #define COM_STARTUP_FILE "startup_ch32v20x_D6.S"
    #define CH32V20x_D6
  #elif defined(CH32V203xB)
    #define COM_STARTUP_FILE "startup_ch32v20x_D8.S"
    #define CH32V20x_D8
  #elif defined(CH32V208)
      #define COM_STARTUP_FILE "startup_ch32v20x_D8W.S"
      #define CH32V20x_D8W
  #else
    #error "Unknow chip!"
  #endif
#else
  #warning "No startup file defined !"
#endif /* CHIP Define */
#endif /* _CH32_DEF_BUILD_ */

