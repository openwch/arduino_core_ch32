#include "../src/USBPD.h"
#include <cassert>

static unsigned apiBeginCalls;
static unsigned apiEndCalls;
static unsigned legacyBeginCalls;
static uint16_t requestedVoltage;
static uint16_t requestedCurrent;

#if defined(TIM_MODULE_ENABLED)
static unsigned timerPauses;
static unsigned timerAttaches;
static unsigned timerDetaches;
static unsigned timerResumes;

HardwareTimer::HardwareTimer(int) {}
void HardwareTimer::setOverflow(uint32_t, int) {}
void HardwareTimer::attachInterrupt(void (*)()) { ++timerAttaches; }
void HardwareTimer::detachInterrupt() { ++timerDetaches; }
void HardwareTimer::pause() { ++timerPauses; }
void HardwareTimer::resume() { ++timerResumes; }
extern "C" void USBPD_Timer_Callback() {}
#endif

extern "C" {
void usbpd_api_begin() { ++apiBeginCalls; }
void usbpd_sink_init() { ++legacyBeginCalls; }
void usbpd_sink_end() { ++apiEndCalls; }
bool usbpd_api_request_voltage(uint16_t voltage, uint16_t current)
{
    requestedVoltage = voltage;
    requestedCurrent = current;
    return true;
}
bool usbpd_api_request_capability(uint8_t, uint16_t) { return true; }
bool usbpd_api_is_ready() { return false; }
USBPD_Status_t usbpd_api_get_status() { return USBPD_STATUS_IDLE; }
uint16_t usbpd_api_get_voltage() { return 0; }
uint16_t usbpd_api_get_current() { return 0; }
bool usbpd_api_has_capabilities() { return false; }
uint8_t usbpd_api_get_capability_count() { return 0; }
bool usbpd_api_get_capability(uint8_t, USBPD_Capability_t *) { return false; }
}

int main()
{
    USBPD.end();
    assert(apiEndCalls == 0);
    assert(USBPD.requestVoltage(9, 0.2f));
    USBPD.begin();
    assert(apiBeginCalls == 1);
    assert(legacyBeginCalls == 0);
    assert(requestedVoltage == 9);
    assert(requestedCurrent == 200);
    USBPD.end();
    USBPD.end();
    assert(apiEndCalls == 1);
    USBPD.begin();
    assert(apiBeginCalls == 2);
    assert(USBPD.requestVoltage(5, 0.5f));
    assert(apiBeginCalls == 2);
    assert(requestedVoltage == 5);
    assert(requestedCurrent == 500);
    USBPD.end();
    assert(apiEndCalls == 2);
    assert(USBPD.requestVoltage(9, 0.2f));
    assert(apiBeginCalls == 3);
#if defined(TIM_MODULE_ENABLED)
    assert(timerAttaches == 3);
    assert(timerResumes == 3);
    assert(timerPauses == 2);
    assert(timerDetaches == 2);
#endif
}
