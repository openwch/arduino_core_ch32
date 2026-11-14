#ifndef USBPD_CAPABILITIES_H
#define USBPD_CAPABILITIES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USBPD_MAX_SOURCE_CAPABILITIES 7

typedef struct
{
    uint16_t Current;
    uint16_t Voltage;
} FixedSourceCap_t;

typedef struct
{
    uint16_t MinVoltage;
    uint16_t MaxVoltage;
    uint16_t Current;
} PPSSourceCap_t;

void usbpd_decode_source_capabilities(const uint8_t *data, uint8_t count,
                                      FixedSourceCap_t *fixed,
                                      PPSSourceCap_t *pps);

#ifdef __cplusplus
}
#endif

#endif
