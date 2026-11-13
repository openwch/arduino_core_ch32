#include <USBPD.h>

#if defined(CH32X035)
#include "src/nativeusb/ch32x035_usbfs_device.h"
#include <stdio.h>

#if F_CPU != 48000000
#error "Native USB serial requires the CH32X035 48 MHz clock."
#endif

static uint8_t reportLine = 0;
static uint8_t lastCount = 0xff;
static uint8_t lastSession = 0xff;
static int lastStatus = -1;
static bool reportPending = false;

static void beginNativeSerial() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_PVDLevelConfig(PWR_PVDLevel_4V0);
  delayMicroseconds(10);
  PWR_VDD supply = PWR_GetFlagStatus(PWR_FLAG_PVDO) == RESET ? PWR_VDD_5V : PWR_VDD_3V3;
  PWR_PVDLevelConfig(PWR_PVDLevel_2V1);

  USBFS_RCC_Init();
  USBFS_Device_Init(ENABLE, supply);
}

static void reportNativeSerial() {
  if (!USBFS_DevEnumStatus) {
    return;
  }

  uint8_t count = USBPD.getCapabilityCount();
  int status = (int)USBPD.getStatus();
  uint8_t session = USBFS_ReportSession;
  if (count != lastCount || status != lastStatus || session != lastSession) {
    lastCount = count;
    lastStatus = status;
    lastSession = session;
    reportLine = 0;
    reportPending = true;
  }
  if (!reportPending || USBFS_Endp_Busy[DEF_UEP3]) {
    return;
  }

  char line[64];
  int length;
  if (reportLine < count) {
    USBPDCapability capability;
    if (USBPD.getCapability(reportLine, capability)) {
      uint32_t minCentivolts = (uint32_t)(capability.minVoltage * 100.0f + 0.5f);
      uint32_t maxCentivolts = (uint32_t)(capability.maxVoltage * 100.0f + 0.5f);
      length = snprintf(line, sizeof(line), "PDO %u: %lu.%02lu-%lu.%02lu V\r\n",
                        reportLine, (unsigned long)(minCentivolts / 100),
                        (unsigned long)(minCentivolts % 100),
                        (unsigned long)(maxCentivolts / 100),
                        (unsigned long)(maxCentivolts % 100));
    } else {
      length = snprintf(line, sizeof(line), "PDO %u: unsupported\r\n", reportLine);
    }
  } else if (USBPD.isReady()) {
    uint32_t centivolts = (uint32_t)(USBPD.getVoltage() * 100.0f + 0.5f);
    length = snprintf(line, sizeof(line), "Negotiated: %lu.%02lu V\r\n",
                      (unsigned long)(centivolts / 100),
                      (unsigned long)(centivolts % 100));
  } else {
    length = snprintf(line, sizeof(line), "PD status: %d, PDOs: %u\r\n",
                      status, count);
  }

  if (length > 0 && length < (int)sizeof(line) &&
      USBFS_Endp_DataUp(DEF_UEP3, (uint8_t *)line, length, DEF_UEP_CPY_LOAD) == 0) {
    if (reportLine >= count) {
      reportPending = false;
    } else {
      ++reportLine;
    }
  }
}
#endif

bool firstCapabilityRequested = false;
#if !defined(CH32X035)
bool capabilitiesListed = false;
bool negotiatedPrinted = false;
#endif

void setup() {
#if defined(CH32X035)
  beginNativeSerial();
#else
  Serial.begin(115200);
#endif
  USBPD.begin();
}

void loop() {
  if (USBPD.hasCapabilities()) {
#if !defined(CH32X035)
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
#endif

    if (!firstCapabilityRequested && USBPD.getCapabilityCount() > 0) {
      // Request the first advertised capability at 1 A.
      if (USBPD.requestCapability(0, 1.0)) {
        firstCapabilityRequested = true;
      }
    }
  }

#if defined(CH32X035)
  reportNativeSerial();
#else
  if (USBPD.isReady() && !negotiatedPrinted) {
    Serial.print("Negotiated: ");
    Serial.print(USBPD.getVoltage());
    Serial.println(" V");
    negotiatedPrinted = true;
  }
#endif
}
