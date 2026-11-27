// Sends a frame every second in loopback mode and prints what comes back.
// Loopback needs no transceiver and no bus: the frame goes from TX to RX inside the chip.
#include <CAN.h>

void setup() {
  Serial.begin(115200);
  if (!CAN.begin(500000) || !CAN.loopback()) {
    Serial.println("CAN start failed");
    while (1) {}
  }
}

void loop() {
  static uint8_t count = 0;
  CAN.beginPacket(0x123);
  CAN.write(count++);
  CAN.print("hi");
  Serial.print(CAN.endPacket() ? "sent" : "send failed");

  int len = CAN.parsePacket();
  if (len) {
    Serial.print(", got id 0x");
    Serial.print(CAN.packetId(), HEX);
    Serial.print(" len ");
    Serial.print(len);
    Serial.print(":");
    while (CAN.available()) {
      Serial.print(" ");
      Serial.print(CAN.read());
    }
  }
  Serial.println();
  delay(1000);
}
