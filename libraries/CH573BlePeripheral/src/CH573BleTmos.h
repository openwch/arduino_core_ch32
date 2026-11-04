#ifndef CH573BLE_TMO_H
#define CH573BLE_TMO_H

#include <Arduino.h>

#include "BLEDevice.h"
#include "BLEDescriptor.h"

#include "CH573BLE_LIB.h"

#include "config.h"


// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD              1600

// How often to perform read rssi event
#define SBP_READ_RSSI_EVT_PERIOD             3200

// Parameter update delay
#define SBP_PARAM_UPDATE_DELAY               6400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)
#define DEFAULT_ADVERTISING_INTERVAL         80

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 6=7.5ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    6

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT         100

// Company Identifier: WCH
#define WCH_COMPANY_ID                       0x07D7

struct ProfileAttrTableFastLutEntry {
    gattAttribute_t *profileAttrPtr;
    uint8_t profileAttrValueLen;
    BLECharacteristic* characteristic;
};

struct NotificationConfigEntry {
    gattCharCfg_t charConfig[PERIPHERAL_MAX_CONNECTION];
    uint16_t valueProfileAttrIndex;
    BLECharacteristic *characteristicPtr;
};

class CH573BleTmos : public BLEDevice
{
  friend class CH573BlePeripheral;
  friend class CH573BleTmos;
public:

// protected:
//     struct localCharacteristicInfo {
//       BLECharacteristic* characteristic;
//       BLEService* service;

//       ble_gatts_char_handles_t handles;
//       bool notifySubscribed;
//       bool indicateSubscribed;
//     };

//     struct remoteServiceInfo {
//       BLERemoteService* service;

//       ble_uuid_t uuid;
//       ble_gattc_handle_range_t handlesRange;
//     };

//     struct remoteCharacteristicInfo {
//       BLERemoteCharacteristic* characteristic;
//       BLERemoteService* service;

//       ble_uuid_t uuid;
//       ble_gatt_char_props_t properties;
//       uint16_t valueHandle;
//     };

    CH573BleTmos();

    virtual ~CH573BleTmos();

    virtual void begin(unsigned char _advertisementDataSize,
                BLEEirData *_advertisementData,
                unsigned char _scanDataSize,
                BLEEirData *_scanData,
                BLELocalAttribute** _localAttributes,
                unsigned char _numLocalAttributes,
                BLERemoteAttribute** _remoteAttributes,
                unsigned char _numRemoteAttributes);

    void bleConnectedCallback(gapEstLinkReqEvent_t *pEvent);
    void bleDisconnectedCallback(gapTerminateLinkEvent_t *pEvent);

    BLEDeviceEventListener* getEventListener();

//     virtual void poll();

//     virtual void end();

//     virtual bool setTxPower(int txPower);
//     virtual void startAdvertising();
//     virtual void disconnect();

    virtual bool updateCharacteristicValue(BLECharacteristic& characteristic);
//     virtual bool broadcastCharacteristic(BLECharacteristic& characteristic);
//     virtual bool canNotifyCharacteristic(BLECharacteristic& characteristic);
//     virtual bool canIndicateCharacteristic(BLECharacteristic& characteristic);

//     virtual bool canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool readRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length);
//     virtual bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
//     virtual bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);

//     virtual void requestAddress();
//     virtual void requestTemperature();
//     virtual void requestBatteryLevel();

//   private:

    // GAP - Advertisement data (max size = 31 bytes, though this is
    // best kept short to conserve power while advertising)
    uint8_t advertData[31];
    uint8_t advertDataLen;
    uint8_t scanRspData[31];
    uint8_t scanRspDataLen;

    gattAttribute_t *profileAttrTbl;
    int profileAttrTblLength;
    unsigned char *uuidTable;
    int uuidTableLength;

    BLELocalAttribute**            localAttributes;
    unsigned char                  numLocalAttributes;


//     bool                              _hasScanData;
//     BLECharacteristic*                _broadcastCharacteristic;

//     uint16_t                          _connectionHandle;
// #if defined(NRF5) || defined(NRF51_S130)
//     uint8_t                           _bondData[((sizeof(ble_gap_enc_key_t) + 3) / 4) * 4]  __attribute__ ((__aligned__(4)));
//     ble_gap_enc_key_t*                _encKey;
// #else
//     uint8_t                           _authStatusBuffer[((sizeof(ble_gap_evt_auth_status_t) + 3) / 4) * 4]  __attribute__ ((__aligned__(4)));
//     ble_gap_evt_auth_status_t*        _authStatus;
// #endif
//     unsigned char                     _txBufferCount;

    // unsigned char                     numLocalCharacteristics;
    // struct localCharacteristicInfo*   localCharacteristicInfo;

//     unsigned char                     _numRemoteServices;
//     struct remoteServiceInfo*         _remoteServiceInfo;
//     unsigned char                     _remoteServiceDiscoveryIndex;
//     unsigned char                     _numRemoteCharacteristics;
//     struct remoteCharacteristicInfo*  _remoteCharacteristicInfo;
//     bool                              _remoteRequestInProgress;

};





#endif // CH573BLE_TMO_H
