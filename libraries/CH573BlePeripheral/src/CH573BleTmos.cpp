#include "CH573BleTmos.h"

#include "CONFIG.h"
#include "devinfoservice.h"

#include "BLEUuid.h"

#include "HAL.h"
#include "peripheral.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t ch573BleTmosProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method);
bStatus_t ch573BleTmosProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
gattServiceCBs_t ch573BleTmosProfileCBs = {
    ch573BleTmosProfile_ReadAttrCB,  // Read callback function pointer
    ch573BleTmosProfile_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

CH573BleTmos *ch573TmosInstance = NULL;
struct ProfileAttrTableFastLutEntry* profileAttrTableFastLut;
int profileAttrTableFastLutLength;
struct NotificationConfigEntry* notificationConfigTable;
int notificationConfigTableLength;


/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
bStatus_t ch573BleTmosProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method)
{
    bStatus_t status = SUCCESS;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if(offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    //check permission from pAttr
    if (pAttr->permissions & GATT_PERMIT_READ) {
        int i;
        for (i = 0; i < profileAttrTableFastLutLength; i++) {
            if (profileAttrTableFastLut[i].profileAttrPtr == pAttr) {
                if (profileAttrTableFastLut[i].profileAttrValueLen > maxLen) {
                    *pLen = maxLen;
                } else {
                    *pLen = profileAttrTableFastLut[i].profileAttrValueLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;
            }
        }
        if (i == profileAttrTableFastLutLength) {
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    } else {
        *pLen = 0;
        return (ATT_ERR_READ_NOT_PERMITTED);
    }

    return (status);
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
bStatus_t ch573BleTmosProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t   notifyApp = 0xFF;

    // If attribute permissions require authorization to write, return error
    if(gattPermitAuthorWrite(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    //check permission from pAttr
    if (pAttr->permissions & GATT_PERMIT_WRITE) {
        if ((pAttr->type.len == ATT_BT_UUID_SIZE) && (pAttr->type.uuid[0]==LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) && (pAttr->type.uuid[1]==HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID))) {
            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY);
        } else{
            int i;
            for (i = 0; i < profileAttrTableFastLutLength; i++) {
                if (profileAttrTableFastLut[i].profileAttrPtr == pAttr) {
                    //Validate the value
                    // Make sure it's not a blob oper
                    if(offset == 0)
                    {
                        if(len > profileAttrTableFastLut[i].profileAttrValueLen)
                        {
                            status = ATT_ERR_INVALID_VALUE_SIZE;
                        }
                    }
                    else
                    {
                        status = ATT_ERR_ATTR_NOT_LONG;
                    }

                    //Write the value
                    if(status == SUCCESS)
                    {
                        //tmos_memcpy(pAttr->pValue, pValue, profileAttrTableFastLut[i].profileAttrValueLen);
                        if (ch573TmosInstance) {
                            BLEDeviceEventListener* _eventListener = ch573TmosInstance->getEventListener();
                            if (_eventListener){
                                _eventListener->BLEDeviceCharacteristicValueChanged(*ch573TmosInstance, *profileAttrTableFastLut[i].characteristic, pValue, len);
                            }
                        }
                        //profileAttrTableFastLut[i].characteristic->setValue(pValue, len);
                        //change to BLEDeviceCharacteristicValueChanged!!!!!!!!!!!
                        //https://github.com/sandeepmistry/arduino-BLEPeripheral/blob/master/src/nRF51822.cpp
                        //notifyApp = SIMPLEPROFILE_CHAR1;
                    }
                    break;
                }
            }
            if (i == profileAttrTableFastLutLength) {
                status = ATT_ERR_ATTR_NOT_FOUND;
            }
        }
    } else {
        return (ATT_ERR_WRITE_NOT_PERMITTED);
    }

    // // If a charactersitic value changed then callback function to notify application of change
    // if((notifyApp != 0xFF) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange)
    // {
    //     simpleProfile_AppCBs->pfnSimpleProfileChange(notifyApp, pValue, len);
    // }
    // In this code, we do not really use simpleProfile_AppCBs, just callback in our own way when needed

    return (status);
}

void ch573BleTmosProfile_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType){
    // Make sure this is not loopback connection
    if(connHandle != LOOPBACK_CONNHANDLE)
    {
        // Reset Client Char Config if connection has dropped
        if((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
           ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) &&
            (!linkDB_Up(connHandle))))
        {
            for (int i = 0; i < notificationConfigTableLength; i++) {
                GATTServApp_InitCharCfg(connHandle, notificationConfigTable[i].charConfig);
            }
        }
    }
}

uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Peripheral";

extern "C" void CH573BleTmosConnectedCallback(gapEstLinkReqEvent_t *pEvent);
extern "C" void CH573BleTmosDisconnectedCallback(gapTerminateLinkEvent_t *pEvent);

void CH573BleTmosConnectedCallback(gapEstLinkReqEvent_t *pEvent) {
    if (ch573TmosInstance) {
        ch573TmosInstance->bleConnectedCallback(pEvent);
    }
}

void CH573BleTmosDisconnectedCallback(gapTerminateLinkEvent_t *pEvent) {
    if (ch573TmosInstance) {
        ch573TmosInstance->bleDisconnectedCallback(pEvent);
    }
}


CH573BleTmos::CH573BleTmos() :
  BLEDevice(),

  advertDataLen(0),
  scanRspDataLen(0)

  //_hasScanData(false)//,
//   _broadcastCharacteristic(NULL),

//   _connectionHandle(BLE_CONN_HANDLE_INVALID),

//   _txBufferCount(0),

//   _numLocalCharacteristics(0),
//   _localCharacteristicInfo(NULL),

//   _numRemoteServices(0),
//   _remoteServiceInfo(NULL),
//   _remoteServiceDiscoveryIndex(0),
//   _numRemoteCharacteristics(0),
//   _remoteCharacteristicInfo(NULL),
//   _remoteRequestInProgress(false)
{
// #if defined(NRF5) || defined(NRF51_S130)
//   this->_encKey = (ble_gap_enc_key_t*)&this->_bondData;
//   memset(&this->_bondData, 0, sizeof(this->_bondData));
// #else
//   this->_authStatus = (ble_gap_evt_auth_status_t*)&this->_authStatusBuffer;
//   memset(&this->_authStatusBuffer, 0, sizeof(this->_authStatusBuffer));
// #endif
    ch573TmosInstance = this;
    profileAttrTableFastLutLength = 0;
    notificationConfigTableLength = 0;
}

CH573BleTmos::~CH573BleTmos() {
  //this->end();
}

void CH573BleTmos::begin(unsigned char _advertisementDataSize,
                      BLEEirData *_advertisementData,
                      unsigned char _scanDataSize,
                      BLEEirData *_scanData,
                      BLELocalAttribute** _localAttributes,
                      unsigned char _numLocalAttributes,
                      BLERemoteAttribute** _remoteAttributes,
                      unsigned char _numRemoteAttributes)
{

     // Initialize the BLE stack
    CH57X_BLEInit();
    // Initialize the hardware abstraction layer
    HAL_Init(); //halTaskID = 2
    // Initialize the GAP role
    // The function is inside the BLE stack binary library
    GAPRole_PeripheralInit();


    






    localAttributes = _localAttributes;
    numLocalAttributes = _numLocalAttributes;

    // prepare advertData
    // flags
    advertDataLen = 0;
    advertData[advertDataLen++] = 0x02; // length
    advertData[advertDataLen++] = GAP_ADTYPE_FLAGS;
    advertData[advertDataLen++] = DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED;

    if (_advertisementDataSize && _advertisementData) {
        for (int i = 0; i < _advertisementDataSize; i++) {
            advertData[advertDataLen + 0] = _advertisementData[i].length + 1;
            advertData[advertDataLen + 1] = _advertisementData[i].type;
            advertDataLen += 2;

            memcpy(&advertData[advertDataLen], _advertisementData[i].data, _advertisementData[i].length);

            advertDataLen += _advertisementData[i].length;
        }
    }


    // prepare scanRspData
    scanRspDataLen = 0;

    if (_scanDataSize && _scanData) {
        for (int i = 0; i < _scanDataSize; i++) {
            scanRspData[scanRspDataLen + 0] = _scanData[i].length + 1;
            scanRspData[scanRspDataLen + 1] = _scanData[i].type;
            scanRspDataLen += 2;

            memcpy(&scanRspData[scanRspDataLen], _scanData[i].data, _scanData[i].length);

            scanRspDataLen += _scanData[i].length;
            //_hasScanData = true;
        }
    }

        //// connection interval range?
        // Tx power level?

    uint8_t  initial_advertising_enable = TRUE;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, scanRspDataLen, scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertDataLen, advertData);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);

        // Set advertising interval
    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t  mitm = TRUE;
        uint8_t  bonding = TRUE;
        uint8_t  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service

    //deal with 1 service for now
    int numberOfCharacteristics = 0;
    int numberOfNotifyCharacteristics = 0;
    int uuidMallocLength = 0;
    for (int i = 0; i < numLocalAttributes; i++) {
        BLELocalAttribute* localAttribute = localAttributes[i];
        BLEUuid uuid = BLEUuid(localAttribute->uuid());
        if (localAttribute->type() == BLETypeCharacteristic) {
            BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
            numberOfCharacteristics++;
            uuidMallocLength += (uuid.length()+3)&0xFC; // Upround to 4 bytes
            uuidMallocLength += (1+3)&0xFC;    // 1 byte for the properties
            if (characteristic->properties() & BLENotify) {
                numberOfNotifyCharacteristics++;
            }
        }
        if (localAttribute->type() == BLETypeService) {
            //BLEService* service = (BLEService*)localAttribute;
            uuidMallocLength += (uuid.length()+3)&0xFC; // Upround to 4 bytes
            uuidMallocLength += (sizeof(gattAttrType_t)+3)&0xFC; // Upround to 4 bytes
        }
        if (localAttribute->type() == BLETypeDescriptor) {
            //BLEDescriptor* descriptor = (BLEDescriptor*)localAttribute;
            //uuid is fixed
        }
    }
    profileAttrTblLength = numLocalAttributes + numberOfCharacteristics+numberOfNotifyCharacteristics;
    profileAttrTbl = (gattAttribute_t *)malloc(sizeof(gattAttribute_t) * profileAttrTblLength);
    uuidTable = (unsigned char *)malloc(uuidMallocLength);
    uuidTableLength = uuidMallocLength;
    profileAttrTableFastLutLength = numberOfCharacteristics;
    profileAttrTableFastLut = (struct ProfileAttrTableFastLutEntry *)malloc(sizeof(ProfileAttrTableFastLutEntry) * profileAttrTableFastLutLength);
    notificationConfigTableLength = numberOfNotifyCharacteristics;
    notificationConfigTable = (struct NotificationConfigEntry *)malloc(sizeof(NotificationConfigEntry) * notificationConfigTableLength);

    int profileAttrTblIndex = 0;
    int profileAttrTableFastLutIndex = 0;
    int notificationConfigTableIndex = 0;
    unsigned char* uuidTableWritePtr = uuidTable;

    for (int i = 0; i < numLocalAttributes; i++) {
        BLELocalAttribute* localAttribute = localAttributes[i];
        BLEUuid uuid = BLEUuid(localAttribute->uuid());
        if (localAttribute->type() == BLETypeCharacteristic) {
            BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
            unsigned char *serviceCharacterUUID = (unsigned char *)uuidTableWritePtr;
            memcpy(serviceCharacterUUID, uuid.data(), uuid.length());
            uuidTableWritePtr += (uuid.length()+3)&0xFC; // Upround to 4 bytes
            unsigned char *charProperty = (unsigned char *)uuidTableWritePtr;
            *charProperty = characteristic->properties();
            uuidTableWritePtr += (1+3)&0xFC;
            unsigned char characteristicPermission = 0;
            if (characteristic->properties() & BLERead) {
                characteristicPermission |= GATT_PERMIT_READ;
            }
            if (characteristic->properties() & BLEWrite) {
                characteristicPermission |= GATT_PERMIT_WRITE;
            }
            if (characteristic->properties() & BLENotify) {
                //characteristicPermission |= GATT_PERMIT_READ; //seem read not really needed
            }
            //not do AUTHEN etc yet

            gattAttribute_t *profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

            profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
            uint8_t *characterUUIDPtr = (uint8_t *)characterUUID;
            memcpy(&profileAttrTblOneEntry->type.uuid, &characterUUIDPtr, sizeof(const uint8_t *));
            profileAttrTblOneEntry->permissions = GATT_PERMIT_READ;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
            memcpy(&profileAttrTblOneEntry->pValue, &charProperty, sizeof(uint8_t *));
            // {{ATT_BT_UUID_SIZE, characterUUID},
            // GATT_PERMIT_READ,
            // 0,
            // &simpleProfileChar1Props},

            profileAttrTblIndex++;

            profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

            profileAttrTblOneEntry->type.len = uuid.length();
            memcpy(&profileAttrTblOneEntry->type.uuid, &serviceCharacterUUID, sizeof(const uint8_t *));
            profileAttrTblOneEntry->permissions = characteristicPermission;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
            unsigned char *charValuePtr = (unsigned char *)characteristic->_value;
            memcpy(&profileAttrTblOneEntry->pValue, &charValuePtr, sizeof(uint8_t *));
            // {
            // {ATT_BT_UUID_SIZE, simpleProfilechar1UUID},
            // GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            // 0,
            // simpleProfileChar1},

            //get the fast lut for characteristic callback
            profileAttrTableFastLut[profileAttrTableFastLutIndex].profileAttrPtr = profileAttrTblOneEntry;
            profileAttrTableFastLut[profileAttrTableFastLutIndex].profileAttrValueLen = characteristic->valueLength();
            profileAttrTableFastLut[profileAttrTableFastLutIndex].characteristic = characteristic;
            profileAttrTableFastLutIndex++;

            profileAttrTblIndex++;

            if (characteristic->properties() & BLENotify) {
                //using error to print sizeof(NotificationConfigEntry)
                NotificationConfigEntry *notificationConfigEntry = &notificationConfigTable[notificationConfigTableIndex];
                notificationConfigEntry->valueProfileAttrIndex = profileAttrTblIndex - 1;   //previous one is characteristic value
                notificationConfigEntry->characteristicPtr = characteristic;

                //add config for notify
                //     {ATT_BT_UUID_SIZE, clientCharCfgUUID},
                //     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                //     0,
                //     (uint8_t *)simpleProfileChar4Config},

                profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

                profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
                uint8_t *clientCharCfgUUIDPtr = (uint8_t *)clientCharCfgUUID;
                memcpy(&profileAttrTblOneEntry->type.uuid, &clientCharCfgUUIDPtr, sizeof(const uint8_t *));
                profileAttrTblOneEntry->permissions = GATT_PERMIT_READ | GATT_PERMIT_WRITE;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
                gattCharCfg_t *charConfigPtr = (gattCharCfg_t *)notificationConfigEntry->charConfig;
                memcpy(&profileAttrTblOneEntry->pValue, &charConfigPtr, sizeof(uint8_t *));

                notificationConfigTableIndex++;
                profileAttrTblIndex++;

                // Initialize Client Characteristic Configuration attributes
                GATTServApp_InitCharCfg( INVALID_CONNHANDLE, charConfigPtr );
            }

        }
        if (localAttribute->type() == BLETypeService) {
            unsigned char *serviceUUID = (unsigned char *)uuidTableWritePtr;
            memcpy(serviceUUID, uuid.data(), uuid.length());
            uuidTableWritePtr += (uuid.length()+3)&0xFC; // Upround to 4 bytes
            gattAttrType_t *seriveAttr = (gattAttrType_t *)uuidTableWritePtr;
            seriveAttr->len = uuid.length();
            //avoid alignment issue, the compiler tried to use sw that does not work with misaligned address, if assign directly
            memcpy(&seriveAttr->uuid, &serviceUUID, sizeof(unsigned char *));
            uuidTableWritePtr += (sizeof(gattAttrType_t)+3)&0xFC;

            gattAttribute_t *profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

            profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
            uint8_t *primaryServiceUUIDPtr = (uint8_t *)primaryServiceUUID;
            memcpy(&profileAttrTblOneEntry->type.uuid, &primaryServiceUUIDPtr, sizeof(const uint8_t *));
            profileAttrTblOneEntry->permissions = GATT_PERMIT_READ;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
            memcpy(&profileAttrTblOneEntry->pValue, &seriveAttr, sizeof(uint8_t *));
            // profileAttrTbl[profileAttrTblIndex] = {
            //     {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
            //     GATT_PERMIT_READ,                       /* permissions */
            //     0,                                      /* handle */
            //     (uint8_t *)&seriveAttr        /* pValue */
            // };
            profileAttrTblIndex++;
        }
        if (localAttribute->type() == BLETypeDescriptor) {
            BLEDescriptor* descriptor = (BLEDescriptor*)localAttribute;
            gattAttribute_t *profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

            profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
            uint8_t *charUserDescUUIDPtr = (uint8_t *)charUserDescUUID;
            memcpy(&profileAttrTblOneEntry->type.uuid, &charUserDescUUIDPtr, sizeof(const uint8_t *));
            profileAttrTblOneEntry->permissions = GATT_PERMIT_READ;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
            ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
            char *descriptorValue = (char *)descriptor->value();
            memcpy(&profileAttrTblOneEntry->pValue, &descriptorValue, descriptor->valueLength());
            //     {ATT_BT_UUID_SIZE, charUserDescUUID},
            //     GATT_PERMIT_READ,
            //     0,
            //     simpleProfileChar1UserDesp},

            profileAttrTblIndex++;
        }
    }

    // Register with Link DB to receive link status change callback
    linkDB_Register(ch573BleTmosProfile_HandleConnStatusCB);

    uint8 status = SUCCESS;

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( profileAttrTbl, profileAttrTblLength,
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &ch573BleTmosProfileCBs );

    // //return ( status );





    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize the peripheral
    Peripheral_Init();



//   for (int i = 0; i < numLocalAttributes; i++) {
//     BLELocalAttribute *localAttribute = localAttributes[i];

//     if (localAttribute->type() == BLETypeCharacteristic) {
//       this->_numLocalCharacteristics++;
//     }
//   }

//   this->_numLocalCharacteristics -= 3; // 0x2a00, 0x2a01, 0x2a05

//   this->_localCharacteristicInfo = (struct localCharacteristicInfo*)malloc(sizeof(struct localCharacteristicInfo) * this->_numLocalCharacteristics);

//   unsigned char localCharacteristicIndex = 0;

//   uint16_t handle = 0;
//   BLEService *lastService = NULL;

//   for (int i = 0; i < numLocalAttributes; i++) {
//     BLELocalAttribute *localAttribute = localAttributes[i];
//     BLEUuid uuid = BLEUuid(localAttribute->uuid());
//     const unsigned char* uuidData = uuid.data();
//     unsigned char value[255];

//     ble_uuid_t nordicUUID;

//     if (uuid.length() == 2) {
//       nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
//       nordicUUID.type = BLE_UUID_TYPE_BLE;
//     } else {
//       unsigned char uuidDataTemp[16];

//       memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

//       nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

//       uuidDataTemp[13] = 0;
//       uuidDataTemp[12] = 0;

//       sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
//     }

//     if (localAttribute->type() == BLETypeService) {
//       BLEService *service = (BLEService *)localAttribute;

//       if (strcmp(service->uuid(), "1800") == 0 || strcmp(service->uuid(), "1801") == 0) {
//         continue; // skip
//       }

//       sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &nordicUUID, &handle);

//       lastService = service;
//     } else if (localAttribute->type() == BLETypeCharacteristic) {
//       BLECharacteristic *characteristic = (BLECharacteristic *)localAttribute;

//       if (strcmp(characteristic->uuid(), "2a00") == 0) {
//         ble_gap_conn_sec_mode_t secMode;
//         BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode); // no security is needed

//         sd_ble_gap_device_name_set(&secMode, characteristic->value(), characteristic->valueLength());
//       } else if (strcmp(characteristic->uuid(), "2a01") == 0) {
//         const uint16_t *appearance = (const uint16_t*)characteristic->value();

//         sd_ble_gap_appearance_set(*appearance);
//       } else if (strcmp(characteristic->uuid(), "2a05") == 0) {
//         // do nothing
//       } else {
//         uint8_t properties = characteristic->properties() & 0xfe;
//         uint16_t valueLength = characteristic->valueLength();

//         this->_localCharacteristicInfo[localCharacteristicIndex].characteristic = characteristic;
//         this->_localCharacteristicInfo[localCharacteristicIndex].notifySubscribed = false;
//         this->_localCharacteristicInfo[localCharacteristicIndex].indicateSubscribed = false;
//         this->_localCharacteristicInfo[localCharacteristicIndex].service = lastService;

//         ble_gatts_char_md_t characteristicMetaData;
//         ble_gatts_attr_md_t clientCharacteristicConfigurationMetaData;
//         ble_gatts_attr_t    characteristicValueAttribute;
//         ble_gatts_attr_md_t characteristicValueAttributeMetaData;

//         memset(&characteristicMetaData, 0, sizeof(characteristicMetaData));

//         memcpy(&characteristicMetaData.char_props, &properties, 1);

//         characteristicMetaData.p_char_user_desc  = NULL;
//         characteristicMetaData.p_char_pf         = NULL;
//         characteristicMetaData.p_user_desc_md    = NULL;
//         characteristicMetaData.p_cccd_md         = NULL;
//         characteristicMetaData.p_sccd_md         = NULL;

//         if (properties & (BLENotify | BLEIndicate)) {
//           memset(&clientCharacteristicConfigurationMetaData, 0, sizeof(clientCharacteristicConfigurationMetaData));

//           BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.read_perm);
//           BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.write_perm);

//           clientCharacteristicConfigurationMetaData.vloc = BLE_GATTS_VLOC_STACK;

//           characteristicMetaData.p_cccd_md = &clientCharacteristicConfigurationMetaData;
//         }

//         memset(&characteristicValueAttributeMetaData, 0, sizeof(characteristicValueAttributeMetaData));

//         if (properties & (BLERead | BLENotify | BLEIndicate)) {
//           if (this->_bondStore) {
//             BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.read_perm);
//           } else {
//             BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.read_perm);
//           }
//         }

//         if (properties & (BLEWriteWithoutResponse | BLEWrite)) {
//           if (this->_bondStore) {
//             BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.write_perm);
//           } else {
//             BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.write_perm);
//           }
//         }

//         characteristicValueAttributeMetaData.vloc       = BLE_GATTS_VLOC_STACK;
//         characteristicValueAttributeMetaData.rd_auth    = 0;
//         characteristicValueAttributeMetaData.wr_auth    = 0;
//         characteristicValueAttributeMetaData.vlen       = !characteristic->fixedLength();

//         for (int j = (i + 1); j < numLocalAttributes; j++) {
//           localAttribute = localAttributes[j];

//           if (localAttribute->type() != BLETypeDescriptor) {
//             break;
//           }

//           BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

//           if (strcmp(descriptor->uuid(), "2901") == 0) {
//             characteristicMetaData.p_char_user_desc        = (uint8_t*)descriptor->value();
//             characteristicMetaData.char_user_desc_max_size = descriptor->valueLength();
//             characteristicMetaData.char_user_desc_size     = descriptor->valueLength();
//           } else if (strcmp(descriptor->uuid(), "2904") == 0) {
//             characteristicMetaData.p_char_pf = (ble_gatts_char_pf_t *)descriptor->value();
//           }
//         }

//         memset(&characteristicValueAttribute, 0, sizeof(characteristicValueAttribute));

//         characteristicValueAttribute.p_uuid       = &nordicUUID;
//         characteristicValueAttribute.p_attr_md    = &characteristicValueAttributeMetaData;
//         characteristicValueAttribute.init_len     = valueLength;
//         characteristicValueAttribute.init_offs    = 0;
//         characteristicValueAttribute.max_len      = characteristic->valueSize();
//         characteristicValueAttribute.p_value      = NULL;

//         sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID, &characteristicMetaData, &characteristicValueAttribute, &this->_localCharacteristicInfo[localCharacteristicIndex].handles);

//         if (valueLength) {
//           for (int j = 0; j < valueLength; j++) {
//             value[j] = (*characteristic)[j];
//           }

//           sd_ble_gatts_value_set(this->_localCharacteristicInfo[localCharacteristicIndex].handles.value_handle, 0, &valueLength, value);
//         }

//         localCharacteristicIndex++;
//       }
//     } else if (localAttribute->type() == BLETypeDescriptor) {
//       BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

//       if (strcmp(descriptor->uuid(), "2901") == 0 ||
//           strcmp(descriptor->uuid(), "2902") == 0 ||
//           strcmp(descriptor->uuid(), "2903") == 0 ||
//           strcmp(descriptor->uuid(), "2904") == 0) {
//         continue; // skip
//       }

//       uint16_t valueLength = descriptor->valueLength();

//       ble_gatts_attr_t descriptorAttribute;
//       ble_gatts_attr_md_t descriptorMetaData;

//       memset(&descriptorAttribute, 0, sizeof(descriptorAttribute));
//       memset(&descriptorMetaData, 0, sizeof(descriptorMetaData));

//       descriptorMetaData.vloc = BLE_GATTS_VLOC_STACK;
//       descriptorMetaData.vlen = (valueLength == descriptor->valueLength()) ? 0 : 1;

//       if (this->_bondStore) {
//         BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&descriptorMetaData.read_perm);
//       } else {
//         BLE_GAP_CONN_SEC_MODE_SET_OPEN(&descriptorMetaData.read_perm);
//       }

//       descriptorAttribute.p_uuid    = &nordicUUID;
//       descriptorAttribute.p_attr_md = &descriptorMetaData;
//       descriptorAttribute.init_len  = valueLength;
//       descriptorAttribute.max_len   = descriptor->valueLength();
//       descriptorAttribute.p_value   = NULL;

//       sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &descriptorAttribute, &handle);

//       if (valueLength) {
//         for (int j = 0; j < valueLength; j++) {
//           value[j] = (*descriptor)[j];
//         }

//         sd_ble_gatts_value_set(handle, 0, &valueLength, value);
//       }
//     }
//   }

//   if ( numRemoteAttributes > 0) {
//     numRemoteAttributes -= 2; // 0x1801, 0x2a05
//   }

//   for (int i = 0; i < numRemoteAttributes; i++) {
//     BLERemoteAttribute *remoteAttribute = remoteAttributes[i];

//     if (remoteAttribute->type() == BLETypeService) {
//       this->_numRemoteServices++;
//     } else if (remoteAttribute->type() == BLETypeCharacteristic) {
//       this->_numRemoteCharacteristics++;
//     }
//   }

//   this->_remoteServiceInfo = (struct remoteServiceInfo*)malloc(sizeof(struct remoteServiceInfo) * this->_numRemoteServices);
//   this->_remoteCharacteristicInfo = (struct remoteCharacteristicInfo*)malloc(sizeof(struct remoteCharacteristicInfo) * this->_numRemoteCharacteristics);

//   BLERemoteService *lastRemoteService = NULL;
//   unsigned char remoteServiceIndex = 0;
//   unsigned char remoteCharacteristicIndex = 0;

//   for (int i = 0; i < numRemoteAttributes; i++) {
//     BLERemoteAttribute *remoteAttribute = remoteAttributes[i];
//     BLEUuid uuid = BLEUuid(remoteAttribute->uuid());
//     const unsigned char* uuidData = uuid.data();

//     ble_uuid_t nordicUUID;

//     if (uuid.length() == 2) {
//       nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
//       nordicUUID.type = BLE_UUID_TYPE_BLE;
//     } else {
//       unsigned char uuidDataTemp[16];

//       memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

//       nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

//       uuidDataTemp[13] = 0;
//       uuidDataTemp[12] = 0;

//       sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
//     }

//     if (remoteAttribute->type() == BLETypeService) {
//       this->_remoteServiceInfo[remoteServiceIndex].service = lastRemoteService = (BLERemoteService *)remoteAttribute;
//       this->_remoteServiceInfo[remoteServiceIndex].uuid = nordicUUID;

//       memset(&this->_remoteServiceInfo[remoteServiceIndex].handlesRange, 0, sizeof(this->_remoteServiceInfo[remoteServiceIndex].handlesRange));

//       remoteServiceIndex++;
//     } else if (remoteAttribute->type() == BLETypeCharacteristic) {
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].characteristic = (BLERemoteCharacteristic *)remoteAttribute;
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].service = lastRemoteService;
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].uuid = nordicUUID;

//       memset(&this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties, 0, sizeof(this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties));
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].valueHandle = 0;

//       remoteCharacteristicIndex++;
//     }
//   }

//   if (this->_bondStore && this->_bondStore->hasData()) {
// #ifdef NRF_51822_DEBUG
//     Serial.println(F("Restoring bond data"));
// #endif
// #if defined(NRF5) || defined(NRF51_S130)
//     this->_bondStore->getData(this->_bondData, 0, sizeof(this->_bondData));
// #else
//     this->_bondStore->getData(this->_authStatusBuffer, 0, sizeof(this->_authStatusBuffer));
// #endif
//   }

//   this->startAdvertising();

// #ifdef __RFduino__
//   RFduinoBLE_enabled = 1;
// #endif
}

bool CH573BleTmos::updateCharacteristicValue(BLECharacteristic& characteristic) {
    bool success = true;

    if (characteristic.properties() & BLENotify) {
        for (int i=0;i<notificationConfigTableLength;i++){
            if (notificationConfigTable[i].characteristicPtr == &characteristic) {
                attHandleValueNoti_t noti;
                // if(len > (peripheralMTU - 3))    //todo: check it
                // {
                //     PRINT("Too large noti\n");
                //     return;
                // }
                noti.len = characteristic.valueLength();
                noti.pValue = (uint8_t *)GATT_bm_alloc(peripheralConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
                bool needFree = true;
                if(noti.pValue)
                {
                    tmos_memcpy(noti.pValue, characteristic.value(), noti.len);
                    uint16_t value = GATTServApp_ReadCharCfg(peripheralConnList.connHandle, notificationConfigTable[i].charConfig);
                    if(value & GATT_CLIENT_CFG_NOTIFY)
                    {
                        noti.handle = profileAttrTbl[notificationConfigTable[i].valueProfileAttrIndex].handle;

                        // Send the notification
                        if (GATT_Notification(peripheralConnList.connHandle, &noti, FALSE) == SUCCESS)
                        {
                            needFree = false;
                        }
                    }
                }
                if(needFree){
                    GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
                }


            }
        }
    }

  return success;

}

void CH573BleTmos::bleConnectedCallback(gapEstLinkReqEvent_t *pEvent) {
    if (this->_eventListener) {
        this->_eventListener->BLEDeviceConnected(*this, pEvent->devAddr);
    }
}

void CH573BleTmos::bleDisconnectedCallback(gapTerminateLinkEvent_t *pEvent) {
    if (this->_eventListener) {
        this->_eventListener->BLEDeviceDisconnected(*this);
    }
}

BLEDeviceEventListener* CH573BleTmos::getEventListener() {
    return this->_eventListener;
}
