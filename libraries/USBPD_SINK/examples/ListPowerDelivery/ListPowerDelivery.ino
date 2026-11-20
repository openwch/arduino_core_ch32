#include <USBPD.h>

bool firstCapabilityRequested = false;
bool capabilitiesListed = false;
bool negotiatedPrinted = false;

void setup() {
  Serial.begin(115200);
  USBPD.begin();
}

void loop() {
  if (USBPD.hasCapabilities()) {
    if (!capabilitiesListed) {
      for (uint8_t i = 0; i < USBPD.getCapabilityCount(); i++) {
        USBPDCapability capability;

        if (USBPD.getCapability(i, capability)) {
          Serial.print("PDO ");
          Serial.print(i);
          Serial.print(": ");
          Serial.print(capability.minVoltage);
          Serial.print("-");
          Serial.print(capability.maxVoltage);
          Serial.println(" V");
        }
      }
      capabilitiesListed = true;
    }

    if (!firstCapabilityRequested && USBPD.getCapabilityCount() > 0) {
      // Request the first advertised capability at 1 A.
      if (USBPD.requestCapability(0, 1.0)) {
        firstCapabilityRequested = true;
      }
    }
  }

  if (USBPD.isReady() && !negotiatedPrinted) {
    Serial.print("Negotiated: ");
    Serial.print(USBPD.getVoltage());
    Serial.println(" V");
    negotiatedPrinted = true;
  }
}
