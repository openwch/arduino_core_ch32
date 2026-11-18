// On CH32X035 select USB CDC Serial and the 48 MHz clock in Tools.
bool wasConnected = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial) {
    if (!wasConnected) {
      Serial.println("USB serial ready");
      wasConnected = true;
    }
    while (Serial.available()) {
      Serial.write(Serial.read());
    }
  } else {
    wasConnected = false;
  }
}
