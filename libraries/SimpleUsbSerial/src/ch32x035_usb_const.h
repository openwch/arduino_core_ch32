#ifndef __CH32X035_USB_CONST_H__
#define __CH32X035_USB_CONST_H__

#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE         64      /* maximum packet size */
#endif

/* USBFS Related Register Macro Definition */

/* R8_USB_CTRL */
#define USBFS_UC_HOST_MODE          0x80
#define USBFS_UC_LOW_SPEED          0x40
#define USBFS_UC_DEV_PU_EN          0x20
#define USBFS_UC_SYS_CTRL_MASK      0x30
#define USBFS_UC_SYS_CTRL0          0x00
#define USBFS_UC_SYS_CTRL1          0x10
#define USBFS_UC_SYS_CTRL2          0x20
#define USBFS_UC_SYS_CTRL3          0x30
#define USBFS_UC_INT_BUSY           0x08
#define USBFS_UC_RESET_SIE          0x04
#define USBFS_UC_CLR_ALL            0x02
#define USBFS_UC_DMA_EN             0x01

/* R8_USB_INT_EN */
#define USBFS_UIE_DEV_SOF           0x80
#define USBFS_UIE_DEV_NAK           0x40
#define USBFS_UIE_FIFO_OV           0x10
#define USBFS_UIE_HST_SOF           0x08
#define USBFS_UIE_SUSPEND           0x04
#define USBFS_UIE_TRANSFER          0x02
#define USBFS_UIE_DETECT            0x01
#define USBFS_UIE_BUS_RST           0x01

/* R8_USB_DEV_AD */
#define USBFS_UDA_GP_BIT            0x80
#define USBFS_USB_ADDR_MASK         0x7F

/* R8_USB_MIS_ST */
#define USBFS_UMS_SOF_PRES          0x80
#define USBFS_UMS_SOF_ACT           0x40
#define USBFS_UMS_SIE_FREE          0x20
#define USBFS_UMS_R_FIFO_RDY        0x10
#define USBFS_UMS_BUS_RESET         0x08
#define USBFS_UMS_SUSPEND           0x04
#define USBFS_UMS_DM_LEVEL          0x02
#define USBFS_UMS_DEV_ATTACH        0x01

/* R8_USB_INT_FG */
#define USBFS_U_IS_NAK              0x80    // RO, indicate current USB transfer is NAK received for USB device mode
#define USBFS_U_TOG_OK              0x40    // RO, indicate current USB transfer toggle is OK
#define USBFS_U_SIE_FREE            0x20    // RO, indicate USB SIE free status
#define USBFS_UIF_FIFO_OV           0x10    // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define USBFS_UIF_HST_SOF           0x08    // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define USBFS_UIF_SUSPEND           0x04    // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define USBFS_UIF_TRANSFER          0x02    // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define USBFS_UIF_DETECT            0x01    // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define USBFS_UIF_BUS_RST           0x01    // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

/* R8_USB_INT_ST */
#define USBFS_SETUP_ACT             0x80      // RO, indicate current USB transfer SETUP is complete
#define USBFS_UIS_TOG_OK            0x40      // RO, indicate current USB transfer toggle is OK
#define USBFS_UIS_TOKEN_MASK        0x30      // RO, bit mask of current token PID code received for USB device mode
#define USBFS_UIS_TOKEN_OUT         0x00
#define USBFS_UIS_TOKEN_SOF         0x10
#define USBFS_UIS_TOKEN_IN          0x20
#define USBFS_UIS_TOKEN_SETUP       0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: SETUP token PID received
#define USBFS_UIS_ENDP_MASK         0x0F      // RO, bit mask of current transfer endpoint number for USB device mode
#define USBFS_UIS_H_RES_MASK        0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received

/* R32_USB_OTG_CR */
#define USBFS_CR_SESS_VTH           0x20
#define USBFS_CR_VBUS_VTH           0x10
#define USBFS_CR_OTG_EN             0x08
#define USBFS_CR_IDPU               0x04
#define USBFS_CR_CHARGE_VBUS        0x02
#define USBFS_CR_DISCHAR_VBUS       0x01

/* R32_USB_OTG_SR */
#define USBFS_SR_ID_DIG             0x08
#define USBFS_SR_SESS_END           0x04
#define USBFS_SR_SESS_VLD           0x02
#define USBFS_SR_VBUS_VLD           0x01

/* R8_UDEV_CTRL */
#define USBFS_UD_PD_DIS             0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define USBFS_UD_DP_PIN             0x20      // ReadOnly: indicate current UDP pin level
#define USBFS_UD_DM_PIN             0x10      // ReadOnly: indicate current UDM pin level
#define USBFS_UD_LOW_SPEED          0x04      // enable USB physical port low speed: 0=full speed, 1=low speed
#define USBFS_UD_GP_BIT             0x02      // general purpose bit
#define USBFS_UD_PORT_EN            0x01      // enable USB physical port I/O: 0=disable, 1=enable

/* R8_UEP4_1_MOD */
#define USBFS_UEP1_RX_EN            0x80      // enable USB endpoint 1 receiving (OUT)
#define USBFS_UEP1_TX_EN            0x40      // enable USB endpoint 1 transmittal (IN)
#define USBFS_UEP1_BUF_MOD          0x10      // buffer mode of USB endpoint 1
// bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define USBFS_UEP4_RX_EN            0x08      // enable USB endpoint 4 receiving (OUT)
#define USBFS_UEP4_TX_EN            0x04      // enable USB endpoint 4 transmittal (IN)
// bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes
#define USBFS_UEP4_BUF_MOD          0x01

/* R8_UEP2_3_MOD */
#define USBFS_UEP3_RX_EN            0x80      // enable USB endpoint 3 receiving (OUT)
#define USBFS_UEP3_TX_EN            0x40      // enable USB endpoint 3 transmittal (IN)
#define USBFS_UEP3_BUF_MOD          0x10      // buffer mode of USB endpoint 3
#define USBFS_UEP2_RX_EN            0x08      // enable USB endpoint 2 receiving (OUT)
#define USBFS_UEP2_TX_EN            0x04      // enable USB endpoint 2 transmittal (IN)
#define USBFS_UEP2_BUF_MOD          0x01      // buffer mode of USB endpoint 2

/* R8_UEP567_MOD */
#define USBFS_UEP5_RX_EN            0x02      // enable USB endpoint 5 receiving (OUT)
#define USBFS_UEP5_TX_EN            0x01      // enable USB endpoint 5 transmittal (IN)
#define USBFS_UEP6_RX_EN            0x08      // enable USB endpoint 6 receiving (OUT)
#define USBFS_UEP6_TX_EN            0x04      // enable USB endpoint 6 transmittal (IN)
#define USBFS_UEP7_RX_EN            0x20      // enable USB endpoint 7 receiving (OUT)
#define USBFS_UEP7_TX_EN            0x10      // enable USB endpoint 7 transmittal (IN)

/* R8_UEPn_TX_CTRL */
#define USBFS_UEP_T_AUTO_TOG        (1<<4)      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define USBFS_UEP_T_TOG             (1<<6)      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define USBFS_UEP_T_RES_MASK        (3<<0)      // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define USBFS_UEP_T_RES_ACK         (0<<1)
#define USBFS_UEP_T_RES_NONE        (1<<0)
#define USBFS_UEP_T_RES_NAK         (1<<1)
#define USBFS_UEP_T_RES_STALL       (3<<0)
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
// host aux setup

/* R8_UEPn_RX_CTRL, n=0-7 */
#define USBFS_UEP_R_AUTO_TOG        (1<<4)      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define USBFS_UEP_R_TOG             (1<<7)      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define USBFS_UEP_R_RES_MASK        (3<<2)      // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define USBFS_UEP_R_RES_ACK         (0<<3)
#define USBFS_UEP_R_RES_NONE        (1<<2)
#define USBFS_UEP_R_RES_NAK         (1<<3)
#define USBFS_UEP_R_RES_STALL       (3<<2)
// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)

/* R8_UHOST_CTRL */
#define USBFS_UH_PD_DIS             0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define USBFS_UH_DP_PIN             0x20      // ReadOnly: indicate current UDP pin level
#define USBFS_UH_DM_PIN             0x10      // ReadOnly: indicate current UDM pin level
#define USBFS_UH_LOW_SPEED          0x04      // enable USB port low speed: 0=full speed, 1=low speed
#define USBFS_UH_BUS_RESET          0x02      // control USB bus reset: 0=normal, 1=force bus reset
#define USBFS_UH_PORT_EN            0x01      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached

/* R32_UH_EP_MOD */
#define USBFS_UH_EP_TX_EN           0x40      // enable USB host OUT endpoint transmittal
#define USBFS_UH_EP_TBUF_MOD        0x10      // buffer mode of USB host OUT endpoint
// bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for transmittal (OUT endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
#define USBFS_UH_EP_RX_EN           0x08      // enable USB host IN endpoint receiving
#define USBFS_UH_EP_RBUF_MOD        0x01      // buffer mode of USB host IN endpoint
// bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (IN endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes

/* R16_UH_SETUP */
#define USBFS_UH_PRE_PID_EN         0x0400      // USB host PRE PID enable for low speed device via hub
#define USBFS_UH_SOF_EN             0x0004      // USB host automatic SOF enable

/* R8_UH_EP_PID */
#define USBFS_UH_TOKEN_MASK         0xF0      // bit mask of token PID for USB host transfer
#define USBFS_UH_ENDP_MASK          0x0F      // bit mask of endpoint number for USB host transfer

/* R8_UH_RX_CTRL */
#define USBFS_UH_R_AUTO_TOG         0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define USBFS_UH_R_TOG              (1<<7)    // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define USBFS_UH_R_RES              0x01      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions

/* R8_UH_TX_CTRL */
#define USBFS_UH_T_AUTO_TOG         0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define USBFS_UH_T_TOG              (1<<6)    // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define USBFS_UH_T_RES              0x01      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions

/* end-point number */
#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07
#define DEF_UEP_NUM                   8


#define USBFSD_UEP_RX_EN              0x08
#define USBFSD_UEP_TX_EN              0x04
#define USBFSD_UEP_BUF_MOD            0x01
#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */

#define USB_IOEN                    0x00000080
#define USB_PHY_V33                 0x00000040
#define UDP_PUE_MASK                0x0000000C
#define UDP_PUE_DISABLE             0x00000000
#define UDP_PUE_35UA                0x00000004
#define UDP_PUE_10K                 0x00000008
#define UDP_PUE_1K5                 0x0000000C

#define UDM_PUE_MASK                0x00000003
#define UDM_PUE_DISABLE             0x00000000
#define UDM_PUE_35UA                0x00000001
#define UDM_PUE_10K                 0x00000002
#define UDM_PUE_1K5                 0x00000003

/* USB standard device request code */
#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS              0x00
#define USB_CLEAR_FEATURE           0x01
#define USB_SET_FEATURE             0x03
#define USB_SET_ADDRESS             0x05
#define USB_GET_DESCRIPTOR          0x06
#define USB_SET_DESCRIPTOR          0x07
#define USB_GET_CONFIGURATION       0x08
#define USB_SET_CONFIGURATION       0x09
#define USB_GET_INTERFACE           0x0A
#define USB_SET_INTERFACE           0x0B
#define USB_SYNCH_FRAME             0x0C
#endif

/* Bit Define for USB Request Type */
#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN              0x80
#define USB_REQ_TYP_OUT             0x00
#define USB_REQ_TYP_READ            0x80
#define USB_REQ_TYP_WRITE           0x00
#define USB_REQ_TYP_MASK            0x60
#define USB_REQ_TYP_STANDARD        0x00
#define USB_REQ_TYP_CLASS           0x20
#define USB_REQ_TYP_VENDOR          0x40
#define USB_REQ_TYP_RESERVED        0x60
#define USB_REQ_RECIP_MASK          0x1F
#define USB_REQ_RECIP_DEVICE        0x00
#define USB_REQ_RECIP_INTERF        0x01
#define USB_REQ_RECIP_ENDP          0x02
#define USB_REQ_RECIP_OTHER         0x03
#define USB_REQ_FEAT_REMOTE_WAKEUP  0x01
#define USB_REQ_FEAT_ENDP_HALT      0x00
#endif

/* R8_USB_INT_ST */
#define USBFS_SETUP_ACT             0x80      // RO, indicate current USB transfer SETUP is complete
#define USBFS_UIS_TOG_OK            0x40      // RO, indicate current USB transfer toggle is OK
#define USBFS_UIS_TOKEN_MASK        0x30      // RO, bit mask of current token PID code received for USB device mode
#define USBFS_UIS_TOKEN_OUT         0x00
#define USBFS_UIS_TOKEN_SOF         0x10
#define USBFS_UIS_TOKEN_IN          0x20
#define USBFS_UIS_TOKEN_SETUP       0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: SETUP token PID received
#define USBFS_UIS_ENDP_MASK         0x0F      // RO, bit mask of current transfer endpoint number for USB device mode
#define USBFS_UIS_H_RES_MASK        0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received


#endif /* __CH32X035_USB_CONST_H__ */
