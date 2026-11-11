#include "CH573BlePeripheral.h"

#include "config.h"
#include "HAL.h"

#include "BLEUuid.h"

extern uint8_t attDeviceName[GAP_DEVICE_NAME_LEN];  // declared in CH573BleTmos.cpp

CH573BlePeripheral::CH573BlePeripheral():
    localName(NULL),
    advertisedServiceUuid(NULL),
    localAttributes(NULL),
    numLocalAttributes(0),
    remoteAttributes(NULL),
    numRemoteAttributes(0),
    // genericAccessService("1800"),
    // deviceNameCharacteristic("2a00", BLERead, 19),
    // appearanceCharacteristic("2a01", BLERead, 2),
    // genericAttributeService("1801"),
    // servicesChangedCharacteristic("2a05", BLEIndicate, 4),

    remoteGenericAttributeService("1801"),
    remoteServicesChangedCharacteristic("2a05", BLEIndicate),

    _central(this)
{
    device = &ch573BleTmos;
    // // Initialize the random number generator
    // tmos_rand();
    // // Initialize the timer
    // TMOS_TimerInit( NULL );
    asm("nop");

    memset(this->_eventHandlers, 0x00, sizeof(this->_eventHandlers));

//   this->setDeviceName(DEFAULT_DEVICE_NAME);
//   this->setAppearance(DEFAULT_APPEARANCE);

   this->device->setEventListener(this);

}

void CH573BlePeripheral::setLocalName(const char *_localName)
{
    localName = _localName;
    int localNameLen = strlen(localName);
    if (localNameLen > (GAP_DEVICE_NAME_LEN - 1)) {
        localNameLen = (GAP_DEVICE_NAME_LEN - 1);
    }
    memcpy(attDeviceName, localName, localNameLen);
    attDeviceName[localNameLen] = 0; // null terminate
    // // Set the local name
    // GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof(localName), (void *)localName );
    // GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof(localName), (void *)localName );
}

void CH573BlePeripheral::begin()
{

    unsigned char advertisementDataSize = 0;

    BLEEirData advertisementData[3];
    BLEEirData scanData;

    scanData.length = 0;

    unsigned char remainingAdvertisementDataLength = BLE_ADVERTISEMENT_DATA_MAX_VALUE_LENGTH + 2;

    // if (this->_serviceSolicitationUuid){
    //     BLEUuid serviceSolicitationUuid = BLEUuid(this->_serviceSolicitationUuid);

    //     unsigned char uuidLength = serviceSolicitationUuid.length();
    //     advertisementData[advertisementDataSize].length = uuidLength;
    //     advertisementData[advertisementDataSize].type = (uuidLength > 2) ? 0x15 : 0x14;

    //     memcpy(advertisementData[advertisementDataSize].data, serviceSolicitationUuid.data(), uuidLength);
    //     advertisementDataSize += 1;
    //     remainingAdvertisementDataLength -= uuidLength + 2;
    // }
    if (this->advertisedServiceUuid){
        BLEUuid advertisedServiceUuid = BLEUuid(this->advertisedServiceUuid);

        unsigned char uuidLength = advertisedServiceUuid.length();
        if (uuidLength + 2 <= remainingAdvertisementDataLength) {
            advertisementData[advertisementDataSize].length = uuidLength;   //add by one in CH573Tmos.begin
            advertisementData[advertisementDataSize].type = (uuidLength > 2) ? GAP_ADTYPE_128BIT_MORE : GAP_ADTYPE_16BIT_MORE;

            memcpy(advertisementData[advertisementDataSize].data, advertisedServiceUuid.data(), uuidLength);
            advertisementDataSize += 1;
            remainingAdvertisementDataLength -= uuidLength + 2;
        }
    }
    // if (this->_manufacturerData && this->_manufacturerDataLength > 0) {
    //     if (remainingAdvertisementDataLength >= 3) {
    //     unsigned char dataLength = this->_manufacturerDataLength;

    //     if (dataLength + 2 > remainingAdvertisementDataLength) {
    //         dataLength = remainingAdvertisementDataLength - 2;
    //     }

    //     advertisementData[advertisementDataSize].length = dataLength;
    //     advertisementData[advertisementDataSize].type = 0xff;

    //     memcpy(advertisementData[advertisementDataSize].data, this->_manufacturerData, dataLength);
    //     advertisementDataSize += 1;
    //     remainingAdvertisementDataLength -= dataLength + 2;
    //     }
    // }

    if (this->localName){
        unsigned char localNameLength = strlen(this->localName);
        scanData.length = localNameLength;   //add by one in CH573Tmos.begin

        if (scanData.length > BLE_SCAN_DATA_MAX_VALUE_LENGTH) {
            scanData.length = BLE_SCAN_DATA_MAX_VALUE_LENGTH;
        }

        scanData.type = (localNameLength > scanData.length) ? 0x08 : 0x09;

        memcpy(scanData.data, this->localName, scanData.length);
    }



   

    // Setup the GAP Peripheral Role Profile
    {
        
        


        if (localAttributes == NULL) {
            initLocalAttributes();
        }
        
        for (int i = 0; i < numLocalAttributes; i++) {
            BLELocalAttribute* localAttribute = localAttributes[i];
            if (localAttribute->type() == BLETypeCharacteristic) {
              BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
              characteristic->setValueChangeListener(*this);
            }
        }
        
        for (int i = 0; i < numRemoteAttributes; i++) {
            BLERemoteAttribute* remoteAttribute = remoteAttributes[i];
            if (remoteAttribute->type() == BLETypeCharacteristic) {
              BLERemoteCharacteristic* remoteCharacteristic = (BLERemoteCharacteristic*)remoteAttribute;
    
              remoteCharacteristic->setValueChangeListener(*this);
            }
        }
        
        if (numRemoteAttributes) {
            addRemoteAttribute(remoteGenericAttributeService);
            addRemoteAttribute(remoteServicesChangedCharacteristic);
        }
        
        device->begin(advertisementDataSize, advertisementData,
                        scanData.length > 0 ? 1 : 0, &scanData,
                        this->localAttributes, this->numLocalAttributes,
                        this->remoteAttributes, this->numRemoteAttributes);
        
        //   this->_device->requestAddress();

    }

}

void CH573BlePeripheral::poll() {
    // device->poll();
    // do nothing
}

void CH573BlePeripheral::disconnect() {
 // this->_device->disconnect();
}

BLECentral CH573BlePeripheral::central() {
 // this->poll();

  return this->_central;
}

bool CH573BlePeripheral::connected() {
  //this->poll();

  return this->_central;
}

void CH573BlePeripheral::setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler eventHandler) {
  if (event < sizeof(this->_eventHandlers)) {
    this->_eventHandlers[event] = eventHandler;
  }
}

void CH573BlePeripheral::setAdvertisedServiceUuid(const char* _advertisedServiceUuid) {
  advertisedServiceUuid = _advertisedServiceUuid;
}

void CH573BlePeripheral::addAttribute(BLELocalAttribute& _attribute) {
  addLocalAttribute(_attribute);
}

void CH573BlePeripheral::addLocalAttribute(BLELocalAttribute& _localAttribute) {
  if (localAttributes == NULL) {
    initLocalAttributes();
  }

  localAttributes[numLocalAttributes] = &_localAttribute;
  numLocalAttributes++;
}

void CH573BlePeripheral::addRemoteAttribute(BLERemoteAttribute& _remoteAttribute) {
  if (remoteAttributes == NULL) {
    remoteAttributes = (BLERemoteAttribute**)malloc(BLERemoteAttribute::numAttributes() * sizeof(BLERemoteAttribute*));
  }

  remoteAttributes[numRemoteAttributes] = &_remoteAttribute;
  numRemoteAttributes++;
}

bool CH573BlePeripheral::characteristicValueChanged(BLECharacteristic& characteristic) {
  return device->updateCharacteristicValue(characteristic);
}


void CH573BlePeripheral::BLEDeviceConnected(BLEDevice& /*device*/, const unsigned char* address) {
  this->_central.setAddress(address);

// #ifdef BLE_PERIPHERAL_DEBUG
//   Serial.print(F("Peripheral connected to central: "));
//   Serial.println(this->_central.address());
// #endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLEConnected];
  if (eventHandler) {
    eventHandler(this->_central);
  }
  asm("nop");
}

void CH573BlePeripheral::BLEDeviceDisconnected(BLEDevice& /*device*/) {
// #ifdef BLE_PERIPHERAL_DEBUG
//   Serial.print(F("Peripheral disconnected from central: "));
//   Serial.println(this->_central.address());
// #endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLEDisconnected];
  if (eventHandler) {
    eventHandler(this->_central);
  }

  this->_central.clearAddress();
}

void CH573BlePeripheral::initLocalAttributes() {
    //numAttributes will increase when a new BLELocalAttribute (Service, Characteris, etc) is created, whereever it is created
    localAttributes = (BLELocalAttribute**)malloc(BLELocalAttribute::numAttributes() * sizeof(BLELocalAttribute*));

    // localAttributes[0] = &genericAccessService;
    // localAttributes[1] = &deviceNameCharacteristic;
    // localAttributes[2] = &appearanceCharacteristic;

    // localAttributes[3] = &genericAttributeService;
    // localAttributes[4] = &servicesChangedCharacteristic;

    numLocalAttributes = 0;
}

void CH573BlePeripheral::BLEDeviceCharacteristicValueChanged(BLEDevice& /*device*/, BLECharacteristic& characteristic, const unsigned char* value, unsigned char valueLength) {
  characteristic.setValue(this->_central, value, valueLength);
}

//run TMOS system process loop

tmosTaskID loop_task_id = INVALID_TASK_ID;
int loopIntervalMs = 20;

extern void CH57X_BLEInit(void);

__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

__attribute__((section(".highcode")))
__attribute__((noinline)) void
Main_Circulation() {
  while (1) {
    TMOS_SystemProcess();
  }
}

#define LOOP_TASK_TMOS_EVT_TEST_1 (0x0001 << 0)

static uint16_t loop_task_process_event(uint8_t task_id, uint16_t events) {
  if (events & LOOP_TASK_TMOS_EVT_TEST_1) {
    loop();
    tmos_start_task(loop_task_id, LOOP_TASK_TMOS_EVT_TEST_1, MS1_TO_SYSTEM_TIME(loopIntervalMs)); //run loop every loopIntervalMs
    return (events ^ LOOP_TASK_TMOS_EVT_TEST_1);
  }
  return 0;
}

void CH573BlePeripheral::startBle(int _loopIntervalMs) {
  loop_task_id = TMOS_ProcessEventRegister(loop_task_process_event);
  tmos_set_event(loop_task_id, LOOP_TASK_TMOS_EVT_TEST_1);

  loopIntervalMs = _loopIntervalMs;
  Main_Circulation();
}
