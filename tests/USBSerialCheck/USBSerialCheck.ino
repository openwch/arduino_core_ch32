bool sent = false;
const char payload[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial) {
    if (!sent) {
      Serial.println("USB serial ready");
      Serial.print(payload);
      Serial.println();
      sent = true;
    }
    while (Serial.available()) {
      Serial.write(Serial.read());
    }
  } else {
    sent = false;
  }
}
