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
requires PD 3.0; active PPS requests are refreshed approximately every
three seconds while the PD timer runs. Current is rounded up to the selected
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
