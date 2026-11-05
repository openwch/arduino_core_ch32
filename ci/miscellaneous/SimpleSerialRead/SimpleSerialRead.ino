int incomingByte = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino is ready. Type something into the Serial Monitor:");
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("I received: ");
    Serial.write(incomingByte);
    Serial.print(" (ASCII value: ");
    Serial.print(incomingByte, DEC);
    Serial.println(")");
  }
}
