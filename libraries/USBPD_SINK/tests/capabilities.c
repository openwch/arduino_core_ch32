#include "../src/usbpd_capabilities.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static void putPdo(uint8_t *data, unsigned index, uint32_t pdo)
{
    for (unsigned byte = 0; byte < 4; ++byte)
        data[index * 4 + byte] = pdo >> (8 * byte);
}

int main(void)
{
    uint8_t data[USBPD_MAX_SOURCE_CAPABILITIES * 4] = {0};
    FixedSourceCap_t fixed[USBPD_MAX_SOURCE_CAPABILITIES];
    PPSSourceCap_t pps[USBPD_MAX_SOURCE_CAPABILITIES];

    for (unsigned i = 0; i < USBPD_MAX_SOURCE_CAPABILITIES; ++i)
        putPdo(data, i, ((100u + i) << 10) | (300u - i));
    usbpd_decode_source_capabilities(data, 7, fixed, pps);
    assert(fixed[6].Voltage == 5300);
    assert(fixed[6].Current == 2940);

    for (unsigned i = 0; i < USBPD_MAX_SOURCE_CAPABILITIES; ++i)
        putPdo(data, i, (3u << 30) | (110u << 17) | (33u << 8) | 60u);
    usbpd_decode_source_capabilities(data, 7, fixed, pps);
    assert(pps[6].MinVoltage == 3300);
    assert(pps[6].MaxVoltage == 11000);
    assert(pps[6].Current == 3000);
    assert(fixed[6].Voltage == 0);

    memset(data, 0, sizeof(data));
    putPdo(data, 0, (100u << 10) | 100u);
    putPdo(data, 1, (3u << 30) | (110u << 17) | (33u << 8) | 60u);
    usbpd_decode_source_capabilities(data, 255, fixed, pps);
    assert(fixed[0].Voltage == 5000 && fixed[0].Current == 1000);
    assert(pps[1].MinVoltage == 3300 && pps[1].MaxVoltage == 11000);
    assert(pps[0].MaxVoltage == 0);

    memset(data, 0, sizeof(data));
    putPdo(data, 0, (1u << 30) | (180u << 10) | 100u);
    putPdo(data, 1, (2u << 30) | (180u << 10) | 100u);
    putPdo(data, 2, (3u << 30) | (1u << 28));
    usbpd_decode_source_capabilities(data, 3, fixed, pps);
    assert(fixed[0].Voltage == 0 && fixed[1].Voltage == 0);
    assert(pps[0].MaxVoltage == 0);
    return 0;
}
