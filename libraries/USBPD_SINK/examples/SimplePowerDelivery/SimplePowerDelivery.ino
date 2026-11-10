#include <USBPD.h>

void setup() {
  // Ask the USB-C source for 9 V and at least 1 A.
  USBPD.requestVoltage(9, 1.0);
}

void loop() {
  if (USBPD.isReady()) {
    // The source accepted the request. This is the contract voltage.
    float voltage = USBPD.getVoltage();
    (void)voltage;
  } else {
    // The requested power is not available yet. Check getStatus() for details.
  }
}
