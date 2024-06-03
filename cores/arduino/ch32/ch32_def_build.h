#ifndef _CH32_DEF_BUILD_
#define _CH32_DEF_BUILD_

#if !defined(COM_STARTUP_FILE)

  #if defined(CH32V00x)
     #define  COM_STARTUP_FILE "startup_ch32v00x.S"
  #elif defined(CH32VM00X)
     #define  COM_STARTUP_FILE "startup_ch32v00X.S"
  #elif defined(CH32X035)
     #define  COM_STARTUP_FILE "startup_ch32x035.S"

  #elif defined(CH32V10x_3V3)
     #define  COM_STARTUP_FILE "startup_ch32v10x_3v3.S"
  #elif defined(CH32V10x_5V)
     #define  COM_STARTUP_FILE "startup_ch32v10x_5v.S"
     
  #elif defined(CH32V203)
    #define COM_STARTUP_FILE "startup_ch32v20x_D6.S"
    #define CH32V20x_D6
  #elif defined(CH32V203xB)
    #define COM_STARTUP_FILE "startup_ch32v20x_D8.S"
    #define CH32V20x_D8
  #elif defined(CH32V208)
      #define COM_STARTUP_FILE "startup_ch32v20x_D8W.S"
      #define CH32V20x_D8W

  #elif defined(CH32V30x_C) 
      #define COM_STARTUP_FILE "startup_ch32v30x_D8C.S"
      #define CH32V30x_D8C
  #elif defined(CH32V30x)
      #define COM_STARTUP_FILE "startup_ch32v30x_D8.S"
      #define CH32V30x_D8

  #elif defined(CH32L10x)  
      #define  COM_STARTUP_FILE  "startup_ch32l103.S"   
  #else
    #error "Unknow chip!"
  #endif
#else
  #warning "No startup file defined !"
#endif /* CHIP Define */
#endif /* _CH32_DEF_BUILD_ */

