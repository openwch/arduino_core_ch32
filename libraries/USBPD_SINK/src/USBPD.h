#ifndef USBPD_H
#define USBPD_H

#include <Arduino.h>
#include "usbpd_api.h"

#ifdef USBPD
#undef USBPD
#endif

/**
 * @file USBPD.h
 * @brief Request and monitor power from a USB-C power source.
 *
 * Request a voltage and current in setup(), then check isReady() in loop().
 * Voltage and current readings describe the negotiated contract, not live
 * electrical measurements.
 */

enum USBPDStatus
{
    USBPD_IDLE = USBPD_STATUS_IDLE,
    USBPD_NEGOTIATING = USBPD_STATUS_NEGOTIATING,
    USBPD_READY = USBPD_STATUS_READY,
    USBPD_NO_SOURCE = USBPD_STATUS_NO_SOURCE,
    USBPD_VOLTAGE_UNAVAILABLE = USBPD_STATUS_VOLTAGE_UNAVAILABLE,
    USBPD_CURRENT_UNAVAILABLE = USBPD_STATUS_CURRENT_UNAVAILABLE,
    USBPD_REJECTED = USBPD_STATUS_REJECTED,
    USBPD_DISCONNECTED = USBPD_STATUS_DISCONNECTED,
    USBPD_ERROR = USBPD_STATUS_ERROR
};

/**
 * @brief One power option advertised by the connected USB-C source.
 *
 * A programmable option can provide any voltage between minVoltage and
 * maxVoltage. Other options provide one fixed voltage.
 */
struct USBPDCapability
{
    float minVoltage;
    float maxVoltage;
    float current;
    bool programmable;
};

/**
 * @brief Simple Arduino interface to USB Power Delivery.
 *
 * A power request is asynchronous. Call isReady() to know when the source
 * has accepted the request and completed the change.
 */
class USBPDClass
{
public:
    /** Start USB Power Delivery. Request functions call this automatically. */
    void begin();

    /**
     * Stop PD processing and clear the reported contract and capabilities.
     * This does not disconnect the cable or guarantee that VBUS returns to 5 V.
     * Call from the main program, not an interrupt. Reconnecting the source
     * may be necessary before starting PD again. A request automatically
     * starts PD again.
     */
    void end();

    /**
     * Request a whole-number voltage and minimum available current.
     * Returns false only when the request values are invalid. Use getStatus()
     * to check whether the source can provide the requested power.
     */
    bool requestVoltage(uint16_t voltage, float current);

    /** Request a listed option by its zero-based index and minimum current. */
    bool requestCapability(uint8_t index, float current);

    /** Return true after the source has activated the requested power. */
    bool isReady();

    /** Explain whether the request is pending, ready, or unavailable. */
    USBPDStatus getStatus();

    /** Return the negotiated contract voltage in volts, or 0 when not ready. */
    float getVoltage();

    /** Return the negotiated contract current in amperes, or 0 when not ready. */
    float getCurrent();

    /** Return true when the source's power options have been received. */
    bool hasCapabilities();

    /** Return the number of source options, including unsupported option types. */
    uint8_t getCapabilityCount();

    /** Read one supported source option; index starts at zero. */
    bool getCapability(uint8_t index, USBPDCapability &capability);

private:
    bool _begun = false;
};

extern USBPDClass USBPD;

#endif
