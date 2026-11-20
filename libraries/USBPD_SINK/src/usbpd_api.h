#ifndef USBPD_API_H
#define USBPD_API_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    USBPD_STATUS_IDLE,
    USBPD_STATUS_NEGOTIATING,
    USBPD_STATUS_READY,
    USBPD_STATUS_NO_SOURCE,
    USBPD_STATUS_VOLTAGE_UNAVAILABLE,
    USBPD_STATUS_CURRENT_UNAVAILABLE,
    USBPD_STATUS_REJECTED,
    USBPD_STATUS_DISCONNECTED,
    USBPD_STATUS_ERROR
} USBPD_Status_t;

typedef struct
{
    uint16_t minVoltage;
    uint16_t maxVoltage;
    uint16_t current;
    bool programmable;
    bool available;
} USBPD_Capability_t;

void usbpd_api_begin(void);
bool usbpd_api_request_voltage(uint16_t voltage, uint16_t current);
bool usbpd_api_request_capability(uint8_t index, uint16_t current);
bool usbpd_api_is_ready(void);
USBPD_Status_t usbpd_api_get_status(void);
uint16_t usbpd_api_get_voltage(void);
uint16_t usbpd_api_get_current(void);
bool usbpd_api_has_capabilities(void);
uint8_t usbpd_api_get_capability_count(void);
bool usbpd_api_get_capability(uint8_t index, USBPD_Capability_t *capability);

#ifdef __cplusplus
}
#endif

#endif
