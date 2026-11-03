// This example shows how to use BLE Peripheral library on CH573
// Underlying stack is TMOS from WCH
// The loop function is called every 20ms by TMOS task scheduler

#include <SimpleUsbSerial.h>
#include <CH573BlePeripheral.h>

CH573BlePeripheral blePeripheral = CH573BlePeripheral();

// Create one or more services. In this case we create a simple service with UUID "1111"
BLEService simpleService = BLEService("1111");


// Create one or more characteristics
// Characteristic 1: read and write with UUID "1111"
BLECharCharacteristic simpleProfilechar1 = BLECharCharacteristic("1111", BLERead | BLEWrite);
BLEDescriptor descriptorChar1 = BLEDescriptor("2901", "char 1, RW");
// Characteristic 2: read only with UUID "2222"
BLECharCharacteristic simpleProfilechar2 = BLECharCharacteristic("2222", BLERead);
BLEDescriptor descriptorChar2 = BLEDescriptor("2901", "char 2, R");
// Characteristic 3: write only with UUID "3333"
BLECharCharacteristic simpleProfilechar3 = BLECharCharacteristic("3333", BLEWrite);
BLEDescriptor descriptorChar3 = BLEDescriptor("2901", "char 3, W");
// Characteristic 4: notify only with UUID "4444"
BLECharCharacteristic simpleProfilechar4 = BLECharCharacteristic("4444", BLENotify);
BLEDescriptor descriptorChar4 = BLEDescriptor("2901", "char 4, N");
// BLEFixedLengthCharacteristic simpleProfilechar5 = BLEFixedLengthCharacteristic("ffe5", BLERead, SIMPLEPROFILE_CHAR5_LEN);  //do it later

#define CONVERT_TO_HEX(x) ((x) > 9 ? (x) + 'A' - 10 : (x) + '0')

void setup() {
  SerialUSB.begin();

  // Setup Serial2 for debug output
  Serial2.setRx(PA_8);
  Serial2.setTx(PA_9);
  Serial2.begin(9600);

  uint8_t MacAddr[6];
  GetMACAddress(MacAddr);

  // setLocalName just copy pointer, so need to be static
  static char deviceName[11];
  strcpy(deviceName, "CH573_0000");
  // set the last 4 digits of the device name to the MAC address
  deviceName[6] = CONVERT_TO_HEX((MacAddr[1] >> 4) & 0x0F);
  deviceName[7] = CONVERT_TO_HEX(MacAddr[1] & 0x0F);
  deviceName[8] = CONVERT_TO_HEX((MacAddr[0] >> 4) & 0x0F);
  deviceName[9] = CONVERT_TO_HEX(MacAddr[0] & 0x0F);

  // set device name and advertised service UUID
  blePeripheral.setLocalName(deviceName);
  blePeripheral.setAdvertisedServiceUuid(simpleService.uuid());

  // add attributes (services, characteristics, descriptors) to peripheral
  // Add service (1111)
  blePeripheral.addAttribute(simpleService);
  // Add characteristic (1111) under service (1111)
  blePeripheral.addAttribute(simpleProfilechar1);
  // descriptor is optional
  blePeripheral.addAttribute(descriptorChar1);
  // Add characteristic (2222) under service (1111)
  blePeripheral.addAttribute(simpleProfilechar2);
  // descriptor is optional
  blePeripheral.addAttribute(descriptorChar2);
  // Add characteristic (3333) under service (1111)
  blePeripheral.addAttribute(simpleProfilechar3);
  // descriptor is optional
  blePeripheral.addAttribute(descriptorChar3);
  // Add characteristic (4444) under service (1111)
  blePeripheral.addAttribute(simpleProfilechar4);
  // descriptor is optional
  blePeripheral.addAttribute(descriptorChar4);
  // blePeripheral.addAttribute(simpleProfilechar5);
  //blePeripheral.addAttribute(descriptor);

  // set initial value
  //characteristic.setValue(0);
  //simpleProfilechar1.setEventHandler(BLEWritten, char1Written);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Supposedly use PA12 as output to drive an LED
  pinMode(PA12, OUTPUT);

  blePeripheral.begin();

  blePeripheral.startBle();
  // Do not put any code after startBle()
}

int loopCounterChar2 = 0;
int char2Value = 0;
int loopCounterChar4 = 0;
int char4Value = 0;

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long previousIncChar2Millis = 0;
  static unsigned long previousIncChar4Millis = 0;

  // Increment char2 value every second
  if (currentMillis - previousIncChar2Millis >= 1000) {
    previousIncChar2Millis = currentMillis;
    char2Value++;
    if (char2Value > 0xFF) {
      char2Value = 0;
    }
    simpleProfilechar2.setValue(char2Value);
    SerialUSB.print("char2Value: ");
    SerialUSB.println(char2Value, HEX);
    SerialUSB.flush();
  }

  // Increment char4 value every 2 seconds
  if (currentMillis - previousIncChar4Millis >= 2000) {
    previousIncChar4Millis = currentMillis;
    char4Value++;
    if (char4Value > 0xFF) {
      char4Value = 0;
    }
    simpleProfilechar4.setValue(char4Value);
    SerialUSB.print("char4Value: ");
    SerialUSB.println(char4Value, HEX);
    SerialUSB.flush();
  }

  // When characteristic 1, print value
  if (simpleProfilechar1.written()) {
    SerialUSB.println("char1 written");
    SerialUSB.println(((int)simpleProfilechar1.value()) & 0xFF, HEX);
    SerialUSB.flush();
  }
  // When characteristic 3, print value and set LED (if value is odd)
  if (simpleProfilechar3.written()) {
    SerialUSB.println("char3 written");
    SerialUSB.println(((int)simpleProfilechar3.value()) & 0xFF, HEX);
    SerialUSB.flush();
    if (simpleProfilechar3.value() & 1) {
      digitalWrite(PA12, HIGH);
    } else {
      digitalWrite(PA12, LOW);
    }
  }
}

// void char1Written(BLECentral& central, BLECharacteristic& characteristic) {
// }

void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler
  SerialUSB.print(F("Connected event, central: "));
  SerialUSB.println(central.address());
  SerialUSB.flush();
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  SerialUSB.print(F("Disconnected event, central: "));
  SerialUSB.println(central.address());
  SerialUSB.flush();
}
