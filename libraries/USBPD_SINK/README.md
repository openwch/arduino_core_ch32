# USB Power Delivery sink

`USBPD` requests power asynchronously from a USB-C PD source. For example:

```cpp
#include <USBPD.h>

void setup() {
  // Request 9 V and reserve 0.2 A. This also starts the PD driver.
  if (!USBPD.requestVoltage(9, 0.2)) {
    // The request arguments were invalid.
  }
}

void loop() {
  if (USBPD.isReady()) {
    // The driver reports that the source accepted the contract.
  } else {
    USBPDStatus status = USBPD.getStatus();
    // Handle unavailable power, rejection, disconnect, or negotiation.
    (void)status;
  }
}
```

`requestVoltage()` accepts whole volts from 5 through 21 and current in
amperes. A `true` return means the arguments were accepted, not that power is
available. Use `getStatus()` and `isReady()` to monitor negotiation. The
driver retries the last request when new source capabilities arrive.

`begin()` starts the driver without requesting power. `hasCapabilities()`,
`getCapabilityCount()` and `getCapability()` enumerate the source's advertised
options; indices are zero-based. `requestCapability()` requests a specific
option, using its minimum voltage for PPS. The voltage request selects an
advertised fixed or PPS option that covers the voltage and current. PPS
requires PD 3.0; active PPS requests are refreshed approximately every three
seconds while the PD timer runs. Current is rounded up to the selected
option's protocol increment.

`getVoltage()` and `getCurrent()` report the driver's last negotiated contract,
not live electrical measurements. They return zero when the driver does not
report a ready contract. A source may reset or behave differently from the
reported state, so use an appropriate meter when verifying VBUS.

Call `USBPD.end()` from the main program to stop PD processing and clear the
reported state. **It does not guarantee that VBUS returns to 5 V**, nor does it
disconnect the cable. Independently verify VBUS and disconnect or renegotiate
safely. A subsequent request starts the driver again, but a physical reconnect
or power cycle may be needed for fresh source capabilities.

Use only a board, power path and connected equipment rated for the requested
voltage and current, including transient margin. Never connect high-voltage
VBUS to a computer or a 3.3 V programmer. The CH32X035 PD timing in this
driver is configured for a 48 MHz system clock and uses TIM3. The listing
example uses the board's hardware `Serial`; it does not provide native USB
serial. MacBook dual-role USB-C ports have shown unstable VBUS in testing and
are not yet verified interoperable. Full PD reset and role-swap handling has
not been established; PPS support is not a claim of full USB-PD compliance.

## Table of contents

- [How to use this library](#how-to-use-this-library)
- [Library functions](#library-functions)
- [Host tests](#host-tests)
- [Features and limitations](#features-and-limitations)

## How to use this library

Include the library header in your sketch. The library provides the global
`USBPD` object; request functions start the driver automatically. You can also
call `USBPD.begin()` to start receiving source capabilities without making a
power request.

```cpp
#include <USBPD.h>

void setup() {
  USBPD.requestVoltage(9, 0.2f);
}

void loop() {
  if (USBPD.isReady()) {
    float voltage = USBPD.getVoltage();
    (void)voltage;
  }
}
```

Complete sketches are in the [examples directory](examples/):
[SimplePowerDelivery](examples/SimplePowerDelivery/SimplePowerDelivery.ino),
[ListPowerDelivery](examples/ListPowerDelivery/ListPowerDelivery.ino), and
[usbpd_sink_request_voltage](examples/usbpd_sink_request_voltage/usbpd_sink_request_voltage.ino).

---

## Library functions

### `USBPD.begin()`

Starts USB Power Delivery processing. It does not make a power request, and
calling it more than once has no additional effect. Request functions call it
automatically.

This method does not return a value.

### `USBPD.end()`

Stops PD processing and clears the reported contract and source capabilities.
Call this from the main program, not an interrupt. It does not disconnect the
cable or guarantee that VBUS returns to 5 V. A physical reconnect or power
cycle may be needed before fresh source capabilities are available.

This method does not return a value.

### `USBPD.requestVoltage(voltage, current)` [[_example_]](examples/SimplePowerDelivery/SimplePowerDelivery.ino)

Requests a whole-number voltage from 5 through 21 V and a minimum current in
amperes. The driver selects a compatible fixed-voltage or programmable (PPS)
source option. The current is rounded up to the option's protocol increment.

The method returns `true` when the request values are accepted; this does not
mean the source has granted the requested power. Check `getStatus()` or
`isReady()` for the negotiation result. Current must be greater than 0 and no
more than 10.23 A.

### `USBPD.requestCapability(index, current)` [[_example_]](examples/ListPowerDelivery/ListPowerDelivery.ino)

Requests a source option by its zero-based index and a minimum current in
amperes. For a programmable (PPS) option, the driver requests its minimum
voltage. Use `hasCapabilities()` and `getCapabilityCount()` before selecting
an option, and use `getCapability()` to inspect it.

The method returns `true` when the request is accepted, not when the source has
granted it. Check `getStatus()` or `isReady()` for the negotiation result.

### `USBPD.isReady()`

Returns `true` when the driver reports that the requested power contract is
ready. This reports protocol state, not an electrical measurement of VBUS.

### `USBPD.getStatus()`

Returns a `USBPDStatus` value describing the request state:

- `USBPD_IDLE` — no active negotiation.
- `USBPD_NEGOTIATING` — the request is being negotiated.
- `USBPD_READY` — the driver reports an accepted power contract.
- `USBPD_NO_SOURCE` — no source capabilities are available.
- `USBPD_VOLTAGE_UNAVAILABLE` — no advertised option can provide the requested voltage.
- `USBPD_CURRENT_UNAVAILABLE` — no suitable option can provide the requested current.
- `USBPD_REJECTED` — the source rejected the request.
- `USBPD_DISCONNECTED` — the source disconnected.
- `USBPD_ERROR` — an error occurred during processing.

### `USBPD.getVoltage()` and `USBPD.getCurrent()`

Return the voltage in volts and current in amperes for the last negotiated
contract. Both return `0` when the driver does not report a ready contract.
These are reported contract values, not live electrical measurements.

### `USBPD.hasCapabilities()` and `USBPD.getCapabilityCount()`

`hasCapabilities()` returns `true` after the source's advertised options have
been received. `getCapabilityCount()` returns the number of advertised
options, including unsupported option types. Capability indices start at zero.

### `USBPD.getCapability(index, capability)` [[_example_]](examples/ListPowerDelivery/ListPowerDelivery.ino)

Reads a source option into a `USBPDCapability` structure. The structure
contains `minVoltage` and `maxVoltage` in volts, `current` in amperes, and a
`programmable` flag. For a fixed-voltage option, the minimum and maximum
voltages are equal.

The method returns `true` when the option can be read; otherwise it returns
`false`.

## Host tests

From the repository root, run:

```sh
c++ -std=c++11 -Wall -Wextra -Werror -I libraries/USBPD_SINK/tests/support libraries/USBPD_SINK/tests/api.cpp libraries/USBPD_SINK/src/USBPD.cpp -o /tmp/usbpd-api-test && /tmp/usbpd-api-test
c++ -std=c++11 -Wall -Wextra -Werror -DTIM_MODULE_ENABLED -I libraries/USBPD_SINK/tests/support libraries/USBPD_SINK/tests/api.cpp libraries/USBPD_SINK/src/USBPD.cpp -o /tmp/usbpd-api-timer-test && /tmp/usbpd-api-timer-test
cc -std=c11 -Wall -Wextra -Werror libraries/USBPD_SINK/tests/capabilities.c libraries/USBPD_SINK/src/usbpd_capabilities.c -o /tmp/usbpd-capabilities-test && /tmp/usbpd-capabilities-test
cc -std=c11 -Wall -Wextra -Werror -Wno-pointer-to-int-cast -Wno-unused-parameter -Wno-unused-variable -I libraries/USBPD_SINK/tests/driver_support libraries/USBPD_SINK/tests/driver.c libraries/USBPD_SINK/src/usbpd_capabilities.c -o /tmp/usbpd-driver-test && /tmp/usbpd-driver-test
```

The driver test uses mock peripheral registers; it does not verify physical
PD signaling, timing, or VBUS.

## Features and limitations

- The API currently targets the CH32X035. PD timing is configured for a 48 MHz system clock and uses TIM3.
- PPS support requires PD 3.0 and is not a claim of full USB-PD compliance. Full PD reset and role-swap handling has not been established.
- The reported voltage and current describe the negotiated contract, not live VBUS measurements. Verify VBUS with appropriate equipment.
- `end()` stops PD processing but cannot guarantee that VBUS returns to 5 V. A physical reconnect or power cycle may be needed to obtain fresh source capabilities.
- MacBook dual-role USB-C ports have shown unstable VBUS in testing and are not yet verified interoperable.
- Use only equipment rated for the requested voltage and current, including transient margin. Never connect high-voltage VBUS to a computer or a 3.3 V programmer.
