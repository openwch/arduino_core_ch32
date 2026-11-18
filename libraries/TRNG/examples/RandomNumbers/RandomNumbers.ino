// Prints a random number from the hardware RNG and a dice roll every second.
#include <TRNG.h>

void setup() {
  Serial.begin(115200);
  TRNG.begin();
}

void loop() {
  Serial.print("random: 0x");
  Serial.print(TRNG.read(), HEX);
  Serial.print("  dice: ");
  Serial.println(TRNG.random(1, 7));
  delay(1000);
}
