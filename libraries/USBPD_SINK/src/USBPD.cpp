#include "USBPD.h"
#include "usbpd_sink.h"

#if defined(TIM_MODULE_ENABLED)
static HardwareTimer usbpdTimer(TIM3);

static void usbpdTimerCallback()
{
    USBPD_Timer_Callback();
    usbpdTimer.resume();
}
#endif

USBPDClass USBPD;

void USBPDClass::begin()
{
    if (_begun)
        return;

    usbpd_api_begin();
#if defined(TIM_MODULE_ENABLED)
    usbpdTimer.setOverflow(5000, MICROSEC_FORMAT);
    usbpdTimer.attachInterrupt(usbpdTimerCallback);
    usbpdTimer.resume();
#endif
    _begun = true;
}

void USBPDClass::end()
{
    if (!_begun)
        return;

#if defined(TIM_MODULE_ENABLED)
    usbpdTimer.pause();
    usbpdTimer.detachInterrupt();
#endif
    usbpd_sink_end();
    _begun = false;
}

bool USBPDClass::requestVoltage(uint16_t voltage, float current)
{
    begin();
    if (!(current > 0.0f) || current > 10.23f)
        return false;
    return usbpd_api_request_voltage(voltage, (uint16_t)(current * 1000.0f));
}

bool USBPDClass::requestCapability(uint8_t index, float current)
{
    begin();
    if (!(current > 0.0f) || current > 10.23f)
        return false;
    return usbpd_api_request_capability(index, (uint16_t)(current * 1000.0f));
}

bool USBPDClass::isReady()
{
    return usbpd_api_is_ready();
}

USBPDStatus USBPDClass::getStatus()
{
    return (USBPDStatus)usbpd_api_get_status();
}

float USBPDClass::getVoltage()
{
    return usbpd_api_get_voltage() / 1000.0f;
}

float USBPDClass::getCurrent()
{
    return usbpd_api_get_current() / 1000.0f;
}

bool USBPDClass::hasCapabilities()
{
    return usbpd_api_has_capabilities();
}

uint8_t USBPDClass::getCapabilityCount()
{
    return usbpd_api_get_capability_count();
}

bool USBPDClass::getCapability(uint8_t index, USBPDCapability &capability)
{
    USBPD_Capability_t value;
    if (!usbpd_api_get_capability(index, &value))
        return false;

    capability.minVoltage = value.minVoltage / 1000.0f;
    capability.maxVoltage = value.maxVoltage / 1000.0f;
    capability.current = value.current / 1000.0f;
    capability.programmable = value.programmable;
    return true;
}
