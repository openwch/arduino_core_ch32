#include "usbpd_capabilities.h"
#include <string.h>

void usbpd_decode_source_capabilities(const uint8_t *data, uint8_t count,
                                      FixedSourceCap_t *fixed,
                                      PPSSourceCap_t *pps)
{
    memset(fixed, 0, sizeof(*fixed) * USBPD_MAX_SOURCE_CAPABILITIES);
    memset(pps, 0, sizeof(*pps) * USBPD_MAX_SOURCE_CAPABILITIES);

    if (count > USBPD_MAX_SOURCE_CAPABILITIES)
        count = USBPD_MAX_SOURCE_CAPABILITIES;

    for (uint8_t i = 0; i < count; ++i)
    {
        const uint8_t *bytes = data + i * 4;
        uint32_t pdo = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                       ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        uint8_t type = pdo >> 30;

        if (type == 0)
        {
            fixed[i].Current = (pdo & 0x3ffu) * 10u;
            fixed[i].Voltage = ((pdo >> 10) & 0x3ffu) * 50u;
        }
        else if (type == 3 && ((pdo >> 28) & 3u) == 0)
        {
            pps[i].Current = (pdo & 0x7fu) * 50u;
            pps[i].MinVoltage = ((pdo >> 8) & 0xffu) * 100u;
            pps[i].MaxVoltage = ((pdo >> 17) & 0xffu) * 100u;
        }
    }
}
