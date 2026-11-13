// clang-format off
#include "USBhandler.h"
// clang-format on

// CDC functions:
void resetCDCParameters();
void setLineCodingHandler();
uint16_t getLineCodingHandler();
void setControlLineStateHandler();
void USB_EP2_IN();
void USB_EP2_OUT();

// clang-format off
//end point ram
#if !(defined (CH585) || defined(CH32V30x))
__attribute__((aligned(4))) uint8_t Ep0Buffer[8];
__attribute__((aligned(4))) uint8_t Ep1Buffer[8];
__attribute__((aligned(4))) uint8_t Ep2Buffer[128];
#else
__attribute__((aligned(16))) uint8_t Ep0Buffer[DEF_USBD_UEP0_SIZE];
__attribute__((aligned(16))) uint8_t Ep1Buffer[DEF_USBD_UEP1_SIZE];
__attribute__((aligned(16))) uint8_t Ep2Buffer[DEF_USBD_UEP2_SIZE*2];
#endif
// clang-format on

uint16_t SetupLen;
uint8_t SetupReq;
volatile uint8_t UsbConfig;
#if defined (CH585) || defined(CH32V30x)
volatile uint8_t UsbDevSpeed;
#endif

uint8_t *pDescr;

void NOP_Process(void) {}

void USBInitForCdc() {

#if defined (CH573) || defined (CH572)
  R8_USB_CTRL = 0x00;
#elif defined (CH585)
  R8_USB2_CTRL = 0x00;
  R8_USBHS_PLL_CTRL = USBHS_PLL_EN;
  R16_PIN_CONFIG |= RB_PIN_USB2_EN;
  R16_U2EP_TX_EN = 0;
  R16_U2EP_RX_EN = 0;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = 0x00;
#elif defined (CH32V30x)
  USBHSD->CONTROL = 0x00;
  USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
#endif

  // Manual flip, OUT transaction returns
  // ACK, IN transaction returns NAK
#if defined (CH573) || defined (CH572)
  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP0_EN;
  R16_U2EP_RX_EN |= RB_EP0_EN;
  R32_U2EP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
  R16_U2EP0_T_LEN  = 0;
  R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
  R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
#elif defined (CH32X035)
  USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
#elif defined (CH32V30x)
  USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
  USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
  USBHSD->UEP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
  USBHSD->UEP0_TX_LEN  = 0;
#endif

  // Endpoint 1, single 64 bytes send buffer
#if defined (CH573) || defined (CH572)
  R8_UEP4_1_MOD = RB_UEP1_TX_EN;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP1_EN;
  R32_U2EP1_MAX_LEN = DEF_USBD_UEP1_SIZE;
  R16_U2EP1_T_LEN = 0;
#elif defined (CH32X035)
  USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
#elif defined (CH32V30x)
  USBHSD->ENDP_CONFIG |= USBHS_UEP1_T_EN;
  USBHSD->UEP1_MAX_LEN = DEF_USBD_UEP1_SIZE;
  USBHSD->UEP1_TX_LEN = 0;
#endif
  // Endpoint 1 automatically flips the sync flag,
  // IN transaction returns NAK
#if defined (CH573) || defined (CH572)
  R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
#elif defined (CH585)
  R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
#elif defined (CH32X035)
  USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
#elif defined (CH32V30x)
  USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
#endif

  // Endpoint 2, single 64 bytes receive buffer, single 64 bytes send buffer
#if defined (CH573) || defined (CH572)
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP2_EN;
  R16_U2EP_RX_EN |= RB_EP2_EN;
  R32_U2EP2_MAX_LEN = DEF_USBD_UEP2_SIZE;
  R16_U2EP2_T_LEN = 0;
#elif defined (CH32X035)
  USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN;
#elif defined (CH32V30x)
  USBHSD->ENDP_CONFIG |= USBHS_UEP2_R_EN | USBHS_UEP2_T_EN;
  USBHSD->UEP2_MAX_LEN = DEF_USBD_UEP2_SIZE;
  USBHSD->UEP2_TX_LEN = 0;
#endif
  // Endpoint 2 automatically flips the sync flag, IN
  // transaction returns NAK, OUT transaction returns ACK
#if defined (CH573) || defined (CH572)
  R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
#elif defined (CH585)
  R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
  R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
#elif defined (CH32X035)
  USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
#elif defined (CH32V30x)
  USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
  USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
#endif

#if defined (CH573) || defined (CH572)
  R16_UEP0_DMA = (uint16_t)(uint32_t)&Ep0Buffer[0];
  R16_UEP1_DMA = (uint16_t)(uint32_t)&Ep1Buffer[0];
  R16_UEP2_DMA = (uint16_t)(uint32_t)&Ep2Buffer[0];
#elif defined (CH585)
  R32_U2EP0_DMA = (uint32_t)&Ep0Buffer[0];  //R32_U2EP0_DMA only has low 17 bit
  R32_U2EP1_RX_DMA = (uint32_t)&Ep1Buffer[0];
  R32_U2EP2_RX_DMA = (uint32_t)&Ep2Buffer[0];
  R32_U2EP2_TX_DMA = (uint32_t)&Ep2Buffer[DEF_USBD_UEP2_SIZE];
#elif defined (CH32X035)
  USBFSD->UEP0_DMA = (uint32_t)&Ep0Buffer[0];
  USBFSD->UEP1_DMA = (uint32_t)&Ep1Buffer[0];
  USBFSD->UEP2_DMA = (uint32_t)&Ep2Buffer[0];
#elif defined (CH32V30x)
  USBHSD->UEP0_DMA = (uint32_t)&Ep0Buffer[0];
  USBHSD->UEP1_RX_DMA = (uint32_t)&Ep1Buffer[0];
  USBHSD->UEP2_RX_DMA = (uint32_t)&Ep2Buffer[0];
  USBHSD->UEP2_TX_DMA = (uint32_t)&Ep2Buffer[DEF_USBD_UEP2_SIZE];
#endif

  // clear interrupt flag
#if defined (CH573) || defined (CH572)
  R8_USB_INT_FG = 0xFF;
#elif defined (CH32X035)
  USBFSD->INT_FG = 0xff;
#elif defined (CH32V30x)
  USBHSD->INT_FG = 0xff;
#endif

  // Device address initialization
#if defined (CH573) || defined (CH572)
  R8_USB_DEV_AD = 0x00;
#elif defined (CH32X035)
  USBFSD->DEV_ADDR = 0x00;
#elif defined (CH32V30x)
  USBHSD->DEV_AD = 0x00;
#endif
  // USB device and internal pull-up enable,
  // automatically return to NAK before interrupt flag
  // is cleared during interrupt
#if defined (CH573) || defined (CH572)
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
#if defined (CH572)
  // The CH572 will pull up the USB lines if RB_PIN_DEBUG_EN is set,
  R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
#endif
#elif defined (CH585)
  R8_USB2_CTRL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;            
  R8_USB2_BASE_MODE = USBHS_UD_SPEED_HIGH;
  R8_USB2_CTRL = USBHS_UD_DEV_EN | USBHS_UD_DMA_EN | USBHS_UD_LPM_EN | USBHS_UD_PHY_SUSPENDM;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
#elif defined (CH32V30x)
  USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
  USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
#endif

  // enable interrupt
#if defined (CH573) || defined (CH572)
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
  PFIC_EnableIRQ(USB_IRQn);
#elif defined (CH585)
  R8_USB2_INT_EN = USBHS_UDIE_BUS_RST | USBHS_UDIE_SUSPEND | USBHS_UDIE_BUS_SLEEP | USBHS_UDIE_LPM_ACT | USBHS_UDIE_TRANSFER | USBHS_UDIE_LINK_RDY;      
  PFIC_EnableIRQ( USB2_DEVICE_IRQn );
#elif defined (CH32X035)
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  NVIC_EnableIRQ( USBFS_IRQn );
#elif defined (CH32V30x)
  USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_BUS_RST | USBHS_UIE_SUSPEND;
  NVIC_EnableIRQ( USBHS_IRQn );
#endif
}

void USB_EP0_SETUP() {
#if defined (CH573) || defined (CH572)
  uint8_t len = R8_USB_RX_LEN;
  len = 8;    //CH573 USB setup packet does not affect R8_USB_RX_LEN. So just force it to 8
#elif defined (CH585)
  uint16_t len = R16_U2EP0_RX_LEN;
#elif defined (CH32X035)
  uint8_t len = USBFSD->RX_LEN;
  len = 8;    //Although not specified in the datasheet, setup packet length seems not correct in CH32X035 either
#elif defined (CH32V30x)
  uint16_t len = USBHSD->RX_LEN;
  len = 8;  //Although not specified in the datasheet, setup packet length seems not correct in CH32V30x either
#endif

  if (len == (sizeof(USB_SETUP_REQ_t))) {
    SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
    len = 0; // Default is success and upload 0 length
    SetupReq = UsbSetupBuf->bRequest;
    if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) !=
        USB_REQ_TYP_STANDARD) // Not standard request
    {

      // here is the commnunication starts, refer to usbFunctionSetup of USBtiny
      // or usb_setup in usbtiny

      switch ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK)) {
      case USB_REQ_TYP_VENDOR: {
        switch (SetupReq) {
        default:
          len = 0xFF; // command not supported
          break;
        }
        break;
      }
      case USB_REQ_TYP_CLASS: {
        switch (SetupReq) {
        case GET_LINE_CODING: // 0x21  currently configured
          len = getLineCodingHandler();
          break;
        case SET_CONTROL_LINE_STATE: // 0x22  generates RS-232/V.24 style
                                      // control signals
          setControlLineStateHandler();
          break;
        case SET_LINE_CODING: // 0x20  Configure
          break;

        default:
          len = 0xFF; // command not supported
          break;
        }
        break;
      }
      default:
        len = 0xFF; // command not supported
        break;
      }

    } else // Standard request
    {
      switch (SetupReq) // Request ccfType
      {
      case USB_GET_DESCRIPTOR:
        switch (UsbSetupBuf->wValueH) {
        case 1: // Device Descriptor
          pDescr = DevDesc; // Put Device Descriptor into outgoing buffer
          len = DevDescLen;
          break;
        case 2: // Configure Descriptor
          #if defined (CH585)
          if( R8_USB2_MIS_ST & USBHS_UDMS_HS_MOD ) {
            UsbDevSpeed = USBHS_SPEED_HIGH;
            //USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
            pDescr = CfgHsDesc;
            len = CfgHsDescLen;
          } else {
            /* Full speed mode */
            UsbDevSpeed = USBHS_SPEED_FULL;
            //USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
            pDescr = DevDesc; // Put Device Descriptor into outgoing buffer
            len = DevDescLen;
          }
          #elif defined (CH32V30x)
            if( ( USBHSD->SPEED_TYPE & USBHS_SPEED_TYPE_MASK ) == USBHS_SPEED_HIGH ) {
              UsbDevSpeed = USBHS_SPEED_HIGH;
              pDescr = CfgHsDesc;
              len = CfgHsDescLen;
            } else {
              UsbDevSpeed = USBHS_SPEED_FULL;
              pDescr = DevDesc; // Put Device Descriptor into outgoing buffer
              len = DevDescLen;
            }
          #else
          pDescr = CfgDesc;
          len = CfgDescLen;
          #endif
          break;
        case 3: //get usb string descriptor
          if (UsbSetupBuf->wValueL == 0) {
            pDescr = LangDes;
            len = LangDesLen;
          } else if (UsbSetupBuf->wValueL == 1) {
            pDescr = Manuf_Des;
            len = Manuf_DesLen;
          } else if (UsbSetupBuf->wValueL == 2) {
              pDescr = Prod_Des;
              len = Prod_DesLen;
          } else if (UsbSetupBuf->wValueL == 3) {
              pDescr = SerDes;
              len = SerDesLen;
          } else if (UsbSetupBuf->wValueL == 4) {
              pDescr = CDC_Des;
              len = CDC_DesLen;
          } else {
              pDescr = SerDes;
              len = SerDesLen;
          }
          break;
        #if defined (CH585) || defined (CH32V30x)
        case 6: //get usb device qualify descriptor
          pDescr = QualDesc;
          len = QualDescLen;
          break;
        case 7: //get usb other speed config descriptor
          if( UsbDevSpeed == USBHS_SPEED_HIGH ) {
              /* High speed mode */
              memcpy( &CfgOtherDesc[2], &CfgDesc[2], CfgDescLen - 2 );
              pDescr = CfgOtherDesc;
              len = CfgDescLen;
          } else if( UsbDevSpeed == USBHS_SPEED_FULL ) {
              /* Full speed mode */
              memcpy( &CfgOtherDesc[2], &CfgHsDesc[2], CfgHsDescLen - 2 );
              pDescr = CfgOtherDesc;
              len = CfgHsDescLen;
          } else {
              len = 0xFF;
          }
          break;
        #endif
        default:
          len = 0xff; // Unsupported descriptors or error
          break;
        }
        if (len != 0xff) {
          if (SetupLen > len) {
            SetupLen = len; // Limit length
          }
          len = SetupLen >= DEFAULT_ENDP0_SIZE
                    ? DEFAULT_ENDP0_SIZE
                    : SetupLen; // transmit length for this packet
          for (uint8_t i = 0; i < len; i++) {
            Ep0Buffer[i] = pDescr[i];
          }
          SetupLen -= len;
          pDescr += len;
        }
        break;
      case USB_SET_ADDRESS:
        SetupLen = UsbSetupBuf->wValueL; // Save the assigned address
        break;
      case USB_GET_CONFIGURATION:
        Ep0Buffer[0] = UsbConfig;
        if (SetupLen >= 1) {
          len = 1;
        }
        break;
      case USB_SET_CONFIGURATION:
        UsbConfig = UsbSetupBuf->wValueL;
        break;
      case USB_GET_INTERFACE:
        break;
      case USB_SET_INTERFACE:
        break;
      case USB_CLEAR_FEATURE: // Clear Feature
        if ((UsbSetupBuf->bRequestType & 0x1F) ==
            USB_REQ_RECIP_DEVICE) // Clear the device featuee.
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x01) {
            if (CfgDesc[ 7 ] & 0x20) {
              // wake up
            } else {
              len = 0xFF; // Failed
            }
          } else {
            len = 0xFF; // Failed
          }
        } else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) ==
                    USB_REQ_RECIP_ENDP) // endpoint
        {
          switch (UsbSetupBuf->wIndexL) {
            case 0x84:
              #if defined (CH573) || defined (CH572)
              R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
              #elif defined (CH585)
              R8_U2EP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #elif defined (CH32X035)
              USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~ (USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
              #elif defined (CH32V30x)
              USBHSD->UEP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #endif
            break;
            case 0x04:
              #if defined (CH573) || defined (CH572)
              R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
              #elif defined (CH585)
              R8_U2EP4_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #elif defined (CH32X035)
              USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
              #elif defined (CH32V30x)
              USBHSD->UEP4_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #endif
            break;
            case 0x83:
              #if defined (CH573) || defined (CH572)
              R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
              #elif defined (CH585)
              R8_U2EP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #elif defined (CH32X035)
              USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
              #elif defined (CH32V30x)
              USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #endif
            break;
            case 0x03:
              #if defined (CH573) || defined (CH572)
              R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
              #elif defined (CH585)
              R8_U2EP3_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #elif defined (CH32X035)
              USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
              #elif defined (CH32V30x)
              USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #endif
            break;
            case 0x82:
              #if defined (CH573) || defined (CH572)
              R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
              #elif defined (CH585)
              R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #elif defined (CH32X035)
              USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
              #elif defined (CH32V30x)
              USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #endif
            break;
            case 0x02:
              #if defined (CH573) || defined (CH572)
              R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
              #elif defined (CH585)
              R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #elif defined (CH32X035)
              USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
              #elif defined (CH32V30x)
              USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #endif
            break;
            case 0x81:
              #if defined (CH573) || defined (CH572)
              R8_UEP1_CTRL = R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
              #elif defined (CH585)
              R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #elif defined (CH32X035)
              USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
              #elif defined (CH32V30x)
              USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
              #endif
            break;
            case 0x01:
              #if defined (CH573) || defined (CH572)
              R8_UEP1_CTRL = R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
              #elif defined (CH585)
              R8_U2EP1_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #elif defined (CH32X035)
              USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
              #elif defined (CH32V30x)
              USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_RES_ACK;
              #endif
            break;
          default:
            len = 0xFF; // Unsupported endpoint
            break;
          }
        } else {
          len = 0xFF; // Unsupported for non-endpoint
        }
        break;
      case USB_SET_FEATURE: // Set Feature
        if ((UsbSetupBuf->bRequestType & 0x1F) ==
            USB_REQ_RECIP_DEVICE) // Set  the device featuee.
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x01) {
            if (CfgDesc[ 7 ] & 0x20) {
              // suspend

              // while ( XBUS_AUX & bUART0_TX );    //Wait till uart0 sending
              // complete SAFE_MOD = 0x55; SAFE_MOD = 0xAA; WAKE_CTRL =
              // bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; //wake up by USB or
              // RXD0/1 signal PCON |= PD; //sleep SAFE_MOD = 0x55; SAFE_MOD =
              // 0xAA; WAKE_CTRL = 0x00;
            } else {
              len = 0xFF; // Failed
            }
          } else {
            len = 0xFF; // Failed
          }
        } else if ((UsbSetupBuf->bRequestType & 0x1F) ==
                    USB_REQ_RECIP_ENDP) // endpoint
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x00) {
            switch (((uint16_t)UsbSetupBuf->wIndexH << 8) |
                    UsbSetupBuf->wIndexL) {
            case 0x84:
              #if defined (CH573) || defined (CH572)
              R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_T_TOG) |
                            UEP_T_RES_STALL; // Set endpoint4 IN STALL
              #elif defined (CH585)
              R8_U2EP4_TX_CTRL = ( R8_U2EP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;

              #elif defined (CH32X035)
              USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_T_TOG) |
                            USBFS_UEP_T_RES_STALL; // Set endpoint4 IN STALL
              #elif defined (CH32V30x)
              USBHSD->UEP4_TX_CTRL = ( USBHSD->UEP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #endif
              break;
            case 0x04:
              #if defined (CH573) || defined (CH572)
              R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint4 OUT Stall
              #elif defined (CH585)
              R8_U2EP4_RX_CTRL = ( R8_U2EP4_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint4 OUT Stall
              #elif defined (CH32V30x)
              USBHSD->UEP4_RX_CTRL = ( USBHSD->UEP4_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #endif
              break;
            case 0x83:
              #if defined (CH573) || defined (CH572)
              R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint3 IN STALL
              #elif defined (CH585)
              R8_U2EP3_TX_CTRL = ( R8_U2EP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint3 IN STALL
              #elif defined (CH32V30x)
              USBHSD->UEP3_TX_CTRL = ( USBHSD->UEP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #endif
              break;
            case 0x03:
              #if defined (CH573) || defined (CH572)
              R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint3 OUT Stall
              #elif defined (CH585)
              R8_U2EP3_RX_CTRL = ( R8_U2EP3_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint3 OUT Stall
              #elif defined (CH32V30x)
              USBHSD->UEP3_RX_CTRL = ( USBHSD->UEP3_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #endif
              break;
            case 0x82:
              #if defined (CH573) || defined (CH572)
              R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint2 IN STALL
              #elif defined (CH585)
              R8_U2EP2_TX_CTRL = ( R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint2 IN STALL
              #elif defined (CH32V30x)
              USBHSD->UEP2_TX_CTRL = ( USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #endif
              break;
            case 0x02:
              #if defined (CH573) || defined (CH572)
              R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint2 OUT Stall
              #elif defined (CH585)
              R8_U2EP2_RX_CTRL = ( R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint2 OUT Stall
              #elif defined (CH32V30x)
              USBHSD->UEP2_RX_CTRL = ( USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #endif
              break;
            case 0x81:
              #if defined (CH573) || defined (CH572)
              R8_UEP1_CTRL = R8_UEP1_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint1 IN STALL
              #elif defined (CH585)
              R8_U2EP1_TX_CTRL = ( R8_U2EP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint1 IN STALL
              #elif defined (CH32V30x)
              USBHSD->UEP1_TX_CTRL = ( USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
              #endif
              break;
            case 0x01:
              #if defined (CH573) || defined (CH572)
              R8_UEP1_CTRL = R8_UEP1_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint1 OUT Stall
              #elif defined (CH585)
              R8_U2EP1_RX_CTRL = ( R8_U2EP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #elif defined (CH32X035)
              USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint1 OUT Stall
              #elif defined (CH32V30x)
              USBHSD->UEP1_RX_CTRL = ( USBHSD->UEP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
              #endif
              break;
            default:
              len = 0xFF; // Failed
              break;
            }
          } else {
            len = 0xFF; // Failed
          }
        } else {
          len = 0xFF; // Failed
        }
        break;
      case USB_GET_STATUS:
        Ep0Buffer[0] = 0x00;
        Ep0Buffer[1] = 0x00;
        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
          //set Ep0Buffer[0] to 2 if sleep.
        }else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
          switch (((uint16_t)UsbSetupBuf->wIndexH << 8) |
                    UsbSetupBuf->wIndexL) {
            case 0x84:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP4_CTRL & MASK_UEP_T_RES) == UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP4_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP4_CTRL_H & USBFS_UEP_T_TOG ) == USBFS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP4_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x04:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP4_CTRL & MASK_UEP_R_RES) == UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP4_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP4_CTRL_H & USBFS_UEP_R_TOG ) == USBFS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP4_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x83:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP3_CTRL & MASK_UEP_T_RES) == UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP3_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP3_CTRL_H & USBFS_UEP_T_TOG ) == USBFS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP3_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x03:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP3_CTRL & MASK_UEP_R_RES) == UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP3_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP3_CTRL_H & USBFS_UEP_R_TOG ) == USBFS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP3_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x82:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP2_CTRL & MASK_UEP_T_RES) == UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP2_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP2_CTRL_H & USBFS_UEP_T_TOG ) == USBFS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP2_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x02:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP2_CTRL & MASK_UEP_R_RES) == UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP2_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP2_CTRL_H & USBFS_UEP_R_TOG ) == USBFS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP2_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x81:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP1_CTRL & MASK_UEP_T_RES) == UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP1_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP1_CTRL_H & USBFS_UEP_T_TOG ) == USBFS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            case 0x01:
              #if defined (CH573) || defined (CH572)
              if ((R8_UEP1_CTRL & MASK_UEP_R_RES) == UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH585)
              if (( R8_U2EP1_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32X035)
              if (( USBFSD->UEP1_CTRL_H & USBFS_UEP_R_TOG ) == USBFS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #elif defined (CH32V30x)
              if (( USBHSD->UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL) {
                Ep0Buffer[0] = 0x01;
              }
              #endif
              break;
            default:
              len = 0xFF; // Unsupported endpoint
              break;
          }
        }
        if (SetupLen >= 2) {
          len = 2;
        } else {
          len = SetupLen;
        }
        break;
      default:
        len = 0xff; // Failed
        break;
      }
    }
  } else {
    len = 0xff; // Wrong packet length
  }
  if (len == 0xff) {
    SetupReq = 0xFF;
    #if defined (CH573) || defined (CH572)
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
    #elif defined (CH585)
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
    #elif defined (CH32X035)
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_STALL | USBFS_UEP_T_RES_STALL; // STALL
    #elif defined (CH32V30x)
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
    #endif
  } else if (len <= DEFAULT_ENDP0_SIZE) // Tx data to host or send 0-length packet
  {
    #if defined (CH573) || defined (CH572)
    R8_UEP0_T_LEN = len;
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH585)
    R16_U2EP0_T_LEN = len;
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    #elif defined (CH32X035)
    USBFSD->UEP0_TX_LEN = len;
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_ACK |
                USBFS_UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH32V30x)
    USBHSD->UEP0_TX_LEN = len;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    #endif
  } else {  //More data needs to be sent, wait for next transaction
    #if defined (CH573) || defined (CH572)
    R8_UEP0_T_LEN = 0; // Tx data to host or send 0-length packet
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH585)
    R16_U2EP0_T_LEN = 0; // Tx data to host or send 0-length packet
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    #elif defined (CH32X035)
    USBFSD->UEP0_TX_LEN = 0; // Tx data to host or send 0-length packet
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_ACK |
                USBFS_UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH32V30x)
    USBHSD->UEP0_TX_LEN = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    #endif
  }
}


void USB_EP0_IN(){
    switch(SetupReq)
    {
        case USB_GET_DESCRIPTOR:
        {
            uint8_t len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                 //send length
            for (uint8_t i=0;i<len;i++){
                Ep0Buffer[i] = pDescr[i];
            }
            //memcpy( Ep0Buffer, pDescr, len );                                  
            SetupLen -= len;
            pDescr += len;
            #if defined (CH573) || defined (CH572)
            R8_UEP0_T_LEN = len;
            R8_UEP0_CTRL ^= RB_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH585)
            R16_U2EP0_T_LEN = len;
            R8_U2EP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;                    //Switch between DATA0 and DATA1
            R8_U2EP0_TX_CTRL = ( R8_U2EP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            // if( USBHS_SetupReqLen == 0 ){
            //    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            // }
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = len;
            USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH32V30x)
            USBHSD->UEP0_TX_LEN = len;
            USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;                    //Switch between DATA0 and DATA1
            //USBHSD->UEP0_TX_CTRL = ( USBHSD->UEP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            #endif
        }
            break;
        case USB_SET_ADDRESS:
            #if defined (CH573) || defined (CH572)
            R8_USB_DEV_AD = R8_USB_DEV_AD & RB_UDA_GP_BIT | SetupLen;
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            #elif defined (CH585)
            R8_USB2_DEV_AD = SetupLen;
            R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
            #elif defined (CH32X035)
            USBFSD->DEV_ADDR = USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT | SetupLen;
            USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
            #elif defined (CH32V30x)
            USBHSD->DEV_AD = SetupLen;
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
            #endif
            break;
        case SET_LINE_CODING: // for some reason CH585 will read this
            uint8_t len = getLineCodingHandler();
            #if defined (CH573) || defined (CH572)
            R8_UEP0_T_LEN = len;
            R8_UEP0_CTRL ^= RB_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH585)
            R16_U2EP0_T_LEN = len;
            R8_U2EP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;                    //Switch between DATA0 and DATA1
            R8_U2EP0_TX_CTRL = ( R8_U2EP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            // if( USBHS_SetupReqLen == 0 ){
            //    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            // }
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = len;
            USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH32V30x)
            USBHSD->UEP0_TX_LEN = len;
            USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;                    //Switch between DATA0 and DATA1
            USBHSD->UEP0_TX_CTRL = ( USBHSD->UEP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            #endif
            break;
        default:
            #if defined (CH573) || defined (CH572)
            R8_UEP0_T_LEN = 0;                                                      // End of transaction
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            #elif defined (CH585)
            R16_U2EP0_T_LEN = 0;
            R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = 0;                                                      // End of transaction
            USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
            #elif defined (CH32V30x)
            USBHSD->UEP0_TX_LEN = 0;
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
            #endif
            break;
    }
    #if defined (CH585)
    R8_U2EP0_TX_CTRL &= ~USBHS_UEP_T_DONE;
    #endif
}

void USB_EP0_OUT(){
    if(SetupReq ==SET_LINE_CODING)  //Set line coding
    {
      #if defined (CH573) || defined (CH572)
        if( R8_USB_INT_FG & RB_U_TOG_OK ){
      #elif defined (CH585)
        // CH585 does not have RB_U_TOG_OK
        {
      #elif defined (CH32X035)
        if( USBFSD->INT_ST & USBFS_UIS_TOG_OK ){
      #elif defined (CH32V30x)
        if( USBHSD->INT_ST & USBHS_UIS_TOG_OK ){
      #endif
          setLineCodingHandler();
          #if defined (CH573) || defined (CH572)
          R8_UEP0_T_LEN = 0;
          R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // send 0-length packet
          #elif defined (CH585)
          R16_U2EP0_T_LEN = 0;
          R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_ACK;
          R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
          #elif defined (CH32X035)
          USBFSD->UEP0_TX_LEN = 0;
          USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_ACK;  // send 0-length packet
          #elif defined (CH32V30x)
          USBHSD->UEP0_TX_LEN = 0;
          USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
          USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
          #endif
        }
    }
    else
    {
      #if defined (CH573) || defined (CH572)
        R8_UEP0_T_LEN = 0;
        R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //Respond Nak
      #elif defined (CH585)
        R16_U2EP0_T_LEN = 0;
        R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
      #elif defined (CH32X035)
        USBFSD->UEP0_TX_LEN = 0;
        USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;  //Respond Nak
      #elif defined (CH32V30x)
        USBHSD->UEP0_TX_LEN = 0;
        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
      #endif
    }
    #if defined (CH585)
    R8_U2EP0_RX_CTRL &= ~USBHS_UEP_R_DONE;
    #endif
}

void USB_EP1_IN() {
#if defined (CH573) || defined (CH572)
    R8_UEP1_T_LEN = 0;
    R8_UEP1_CTRL = R8_UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Default NAK
#elif defined (CH585)
    R16_U2EP1_T_LEN = 0;
    R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;     
    R8_U2EP1_TX_CTRL &= ~USBHS_UEP_T_DONE;                
#elif defined (CH32X035)
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Default NAK
#elif defined (CH32V30x)
    USBHSD->UEP1_TX_LEN = 0;
    USBHSD->UEP1_TX_CTRL = USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK | USBHS_UEP_T_RES_NAK; // Default NAK
#endif
}

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
#if defined (CH573) || defined (CH572)
void USB_IRQHandler(void) {
#elif defined (CH585)
void USB2_DEVICE_IRQHandler(void) {
#elif defined (CH32X035)
void USBFS_IRQHandler(void) {
#elif defined (CH32V30x)
void USBHS_IRQHandler(void) {
#endif

#if defined (CH573) || defined (CH572)
  if (R8_USB_INT_FG & RB_UIF_TRANSFER) {
#elif defined (CH585)
  if (R8_USB2_INT_FG & USBHS_UDIF_TRANSFER) {
#elif defined (CH32X035)
  if (USBFSD->INT_FG & USBFS_UIF_TRANSFER) {
#elif defined (CH32V30x)
  if (USBHSD->INT_FG & (USBHS_UIF_TRANSFER | USBHS_UIF_SETUP_ACT)) { // CH32V30x has setup bit in transfer interrupt
#endif
    // there is difference in CH573 from CH552, there is RB_UIS_SETUP_ACT on bit7 of R8_USB_INT_ST
    // and it does not affect RB_UIS_TOG_OK, MASK_UIS_TOKEN , MASK_UIS_ENDP and R8_USB_RX_LEN
#if defined (CH573) || defined (CH572)
    if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) {
#elif defined (CH585)
    // CH585 has RB_UEP_R_SETUP_IS in R8_U2EP0_RX_CTRL
    if ( ((R8_USB2_INT_ST & (USBHS_UDIS_EP_ID_MASK | USBHS_UDIS_EP_DIR)) == 0) && (R8_U2EP0_RX_CTRL & USBHS_UEP_R_SETUP_IS) ) {
#elif defined (CH32X035)
    if (USBFSD->INT_ST & USBFS_SETUP_ACT) {
#elif defined (CH32V30x)
    if (USBHSD->INT_FG & USBHS_UIF_SETUP_ACT) {
#endif
        EP0_SETUP_Callback();
#if defined (CH32V30x)
        USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;  // Clear interrupt flag, as only CH32V30x has setup bit in interrupt flag register
#endif
    }else{
#if defined (CH573) || defined (CH572)
        uint8_t callIndex = R8_USB_INT_ST & MASK_UIS_ENDP;
        switch (R8_USB_INT_ST & MASK_UIS_TOKEN) {
#elif defined (CH585)
        uint8_t callIndex = R8_USB2_INT_ST & USBHS_UDIS_EP_ID_MASK;
        switch ((R8_USB2_INT_ST & USBHS_UDIS_EP_DIR) | (R8_USB2_INT_FG & USBHS_UDIF_RX_SOF & 0)) {
#elif defined (CH32X035)
        uint8_t callIndex = USBFSD->INT_ST & USBFS_UIS_ENDP_MASK;
        switch (USBFSD->INT_ST & USBFS_UIS_TOKEN_MASK) {
#elif defined (CH32V30x)
        uint8_t callIndex = USBHSD->INT_ST & USBHS_UIS_ENDP_MASK;
        switch (USBHSD->INT_ST & USBHS_UIS_TOKEN_MASK) {
#endif

#if defined (CH573) || defined (CH572)
          case UIS_TOKEN_OUT:
#elif defined (CH585)
          case 0: //RB_UDIS_EP_DIR is 0
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_OUT:
#elif defined (CH32V30x)
          case USBHS_UIS_TOKEN_OUT:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_OUT_Callback();
                  break;
                case 1:
                  EP1_OUT_Callback();
                  break;
                case 2:
                  EP2_OUT_Callback();
                  break;
                case 3:
                  EP3_OUT_Callback();
                  break;
                case 4:
                  EP4_OUT_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
#if defined (CH573) || defined (CH572)
          case UIS_TOKEN_SOF:
#elif defined (CH585)
          case 0xFFFF: //USBHS_UDIF_RX_SOF seems not used in EVT example
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_SOF:
#elif defined (CH32V30x)
          case USBHS_UIS_TOKEN_SOF:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_SOF_Callback();
                  break;
                case 1:
                  EP1_SOF_Callback();
                  break;
                case 2:
                  EP2_SOF_Callback();
                  break;
                case 3:
                  EP3_SOF_Callback();
                  break;
                case 4:
                  EP4_SOF_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
#if defined (CH573) || defined (CH572)
          case UIS_TOKEN_IN:
#elif defined (CH585)
          case USBHS_UDIS_EP_DIR: // RB_UDIS_EP_DIR is 1
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_IN:
#elif defined (CH32V30x)
          case USBHS_UIS_TOKEN_IN:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_IN_Callback();
                  break;
                case 1:
                  EP1_IN_Callback();
                  break;
                case 2:
                  EP2_IN_Callback();
                  break;
                case 3:
                  EP3_IN_Callback();
                  break;
                case 4:
                  EP4_IN_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
        }
    }
#if defined (CH573) || defined (CH572)
    R8_USB_INT_FG = RB_UIF_TRANSFER;  // Clear interrupt flag
#elif defined (CH585)
    R8_USB2_INT_FG = USBHS_UDIF_TRANSFER;  // Clear interrupt flag
#elif defined (CH32X035)
    USBFSD->INT_FG = USBFS_UIF_TRANSFER;  // Clear interrupt flag
#elif defined (CH32V30x)
    USBHSD->INT_FG = USBHS_UIF_TRANSFER;  // Clear interrupt flag
#endif
  }

#if defined (CH585)
  if( R8_USB2_INT_FG & USBHS_UDIF_LINK_RDY ){
    R8_USB2_INT_FG = USBHS_UDIF_LINK_RDY;
  }
#endif

    // Device mode USB bus reset
#if defined (CH573) || defined (CH572)
    if (R8_USB_INT_FG & RB_UIF_BUS_RST){
#elif defined (CH585)
    if (R8_USB2_INT_FG & USBHS_UDIF_BUS_RST){
#elif defined (CH32X035)
    if (USBFSD->INT_FG & USBFS_UIF_BUS_RST){
#elif defined (CH32V30x)
    if (USBHSD->INT_FG & USBHS_UIF_BUS_RST){
#endif
        // Manual flip, OUT transaction returns
        // ACK, IN transaction returns NAK
        #if defined (CH573) || defined (CH572)
          R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        #elif defined (CH585)
          R16_U2EP0_T_LEN  = 0;
          R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
          R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
        #elif defined (CH32X035)
          USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
        #elif defined (CH32V30x)
          USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
          USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
        #endif
        // Endpoint 1 automatically flips the sync flag,
        // IN transaction returns NAK
        #if defined (CH573) || defined (CH572)
          R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
        #elif defined (CH585)
          R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
        #elif defined (CH32X035)
          USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
        #elif defined (CH32V30x)
          USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
        #endif
        // Endpoint 2 automatically flips the sync flag, IN
        // transaction returns NAK, OUT transaction returns ACK
        #if defined (CH573) || defined (CH572)
          R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
        #elif defined (CH585)
          R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
          R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
        #elif defined (CH32X035)
          USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
        #elif defined (CH32V30x)
          USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
          USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
        #endif

        #if defined (CH573) || defined (CH572)
        R8_USB_DEV_AD = 0x00;
        R8_USB_INT_FG = RB_UIF_SUSPEND|RB_UIF_TRANSFER|RB_UIF_BUS_RST; // Clear interrupt flag
        #elif defined (CH585)
        R8_USB2_DEV_AD = 0;
        R8_USB2_INT_FG = USBHS_UDIF_BUS_RST;
        #elif defined (CH32X035)
        USBFSD->DEV_ADDR = 0x00;
        USBFSD->INT_FG = USBFS_UIF_SUSPEND|USBFS_UIF_TRANSFER|USBFS_UIF_BUS_RST; // Clear interrupt flag
        #elif defined (CH32V30x)
        USBHSD->DEV_AD = 0x00;
        USBHSD->INT_FG = USBHS_UIF_BUS_RST; // Clear interrupt flag
        #endif

        UsbConfig = 0;

        resetCDCParameters();
    }

    // USB bus suspend / wake up
#if defined (CH573) || defined (CH572)
    if (R8_USB_INT_FG & RB_UIF_SUSPEND) {
#elif defined (CH585)
    if (R8_USB2_INT_FG & USBHS_UDIF_SUSPEND) {
#elif defined (CH32X035)
    if (USBFSD->INT_FG & USBFS_UIF_SUSPEND) {
#elif defined (CH32V30x)
    if (USBHSD->INT_FG & USBHS_UIF_SUSPEND) {
#endif

#if defined (CH573) || defined (CH572)
        R8_USB_INT_FG = RB_UIF_SUSPEND; // Clear interrupt flag
#elif defined (CH585)
        R8_USB2_INT_FG = USBHS_UDIF_SUSPEND; // Clear interrupt flag
#elif defined (CH32X035)
        USBFSD->INT_FG = USBFS_UIF_SUSPEND; // Clear interrupt flag
#elif defined (CH32V30x)
        USBHSD->INT_FG = USBHS_UIF_SUSPEND; // Clear interrupt flag
#endif

#if defined (CH573) || defined (CH572)
        if (R8_USB_MIS_ST & RB_UMS_SUSPEND) { // Suspend
#elif defined (CH585)
        if (R8_USB2_MIS_ST & USBHS_UDMS_SUSPEND) { // Suspend
#elif defined (CH32X035)
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) { // Suspend
#elif defined (CH32V30x)
        if (USBHSD->MIS_ST & USBHS_UMS_SUSPEND) { // Suspend
#endif
            //don't need to do anything
        }else{
            //don't need to do anything
        }
    }
}

