#ifndef CH573BLEPERIPHERAL_H
#define CH573BLEPERIPHERAL_H

#include <Arduino.h>
#include "CH573BLE_LIB.h"

// #include "BLEBondStore.h"
#include "BLECentral.h"
// #include "BLEConstantCharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEDevice.h"
// #include "BLEFixedLengthCharacteristic.h"
#include "BLELocalAttribute.h"
// #include "BLEProgmemConstantCharacteristic.h"
// #include "BLERemoteAttribute.h"
// #include "BLERemoteCharacteristic.h"
// #include "BLERemoteService.h"
#include "BLEService.h"
#include "BLETypedCharacteristics.h"

#include "CH573BleTmos.h"

enum BLEPeripheralEvent {
  BLEConnected = 0,
  BLEDisconnected = 1,
  BLEBonded = 2,
  BLERemoteServicesDiscovered = 3
};

typedef void (*BLEPeripheralEventHandler)(BLECentral& central);

class CH573BlePeripheral : public BLEDeviceEventListener, public BLECharacteristicValueChangeListener, public BLERemoteCharacteristicValueChangeListener
{
  public:
    CH573BlePeripheral();


    void begin();
    void poll();
    // void end();

    void setLocalName(const char *_localName);
    void setAdvertisedServiceUuid(const char* _advertisedServiceUuid);

    void addAttribute(BLELocalAttribute& _attribute);
    void addLocalAttribute(BLELocalAttribute& _localAttribute);
    void addRemoteAttribute(BLERemoteAttribute& _remoteAttribute);

    void disconnect();

    void startBle(int _loopIntervalMs = 20);

    BLECentral central();
    bool connected();

    void setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler eventHandler);
  
  private:
    void initLocalAttributes();

  protected:
    bool characteristicValueChanged(BLECharacteristic& characteristic);
    virtual void BLEDeviceConnected(BLEDevice& device, const unsigned char* address);
    virtual void BLEDeviceDisconnected(BLEDevice& device);

    virtual void BLEDeviceCharacteristicValueChanged(BLEDevice& device, BLECharacteristic& characteristic, const unsigned char* value, unsigned char valueLength);

  public:

    BLEDevice*                     device;
    CH573BleTmos                   ch573BleTmos;

    const char*                    localName;
    const char*                    advertisedServiceUuid;

    BLELocalAttribute**            localAttributes;
    unsigned char                  numLocalAttributes;
    BLERemoteAttribute**           remoteAttributes;
    unsigned char                  numRemoteAttributes;

    // seems not needed for TMOS
    // BLEService                     genericAccessService;
    // BLECharacteristic              deviceNameCharacteristic;
    // BLECharacteristic              appearanceCharacteristic;
    // BLEService                     genericAttributeService;
    // BLECharacteristic              servicesChangedCharacteristic;

    BLERemoteService               remoteGenericAttributeService;
    BLERemoteCharacteristic        remoteServicesChangedCharacteristic;

    BLECentral                     _central;
    BLEPeripheralEventHandler      _eventHandlers[4];


};




#endif // CH573BLEPERIPHERAL_H

