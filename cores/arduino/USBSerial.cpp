#include "Arduino.h"

#if defined(CH32X035) && defined(CH32X035_USB_CDC)

#if F_CPU != 48000000
#error "USB CDC Serial on CH32X035 requires the 48 MHz clock."
#endif

#include "USBSerial.h"
#include "usbfs_x035/ch32x035_usbfs_device.h"

USBSerial SerialUSB;
static bool started;

void USBSerial::begin(unsigned long baud)
{
    (void)baud;
    if (started)
        return;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_PVDLevelConfig(PWR_PVDLevel_4V0);
    delayMicroseconds(10);
    PWR_VDD supply = PWR_GetFlagStatus(PWR_FLAG_PVDO) == RESET ? PWR_VDD_5V : PWR_VDD_3V3;
    PWR_PVDLevelConfig(PWR_PVDLevel_2V1);
    UART2_ParaInit(1);
    USBFS_RCC_Init();
    USBFS_Device_Init(ENABLE, supply);
    started = true;
}

void USBSerial::end()
{
    if (!started)
        return;
    USBFS_Device_Init(DISABLE, PWR_VDD_3V3);
    USBFS_DevEnumStatus = 0;
    USBFS_LineOpen = 0;
    USBFS_RxLength = 0;
    USBFS_RxOffset = 0;
    started = false;
}

int USBSerial::available()
{
    return USBFS_RxLength - USBFS_RxOffset;
}

int USBSerial::peek()
{
    return available() ? USBFS_EP2_Buf[USBFS_RxOffset] : -1;
}

int USBSerial::read()
{
    if (!available())
        return -1;
    int value = USBFS_EP2_Buf[USBFS_RxOffset++];
    if (USBFS_RxOffset == USBFS_RxLength) {
        USBFS_RxOffset = 0;
        USBFS_RxLength = 0;
        USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK) |
                              USBFS_UEP_R_RES_ACK;
    }
    return value;
}

size_t USBSerial::write(uint8_t value)
{
    return write(&value, 1);
}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{
    size_t written = 0;
    while (written < size) {
        unsigned long start = millis();
        while (USBFS_Endp_Busy[DEF_UEP3]) {
            if (!*this || millis() - start >= 250) {
                setWriteError();
                return written;
            }
            yield();
        }
        if (!*this) {
            setWriteError();
            break;
        }
        size_t length = size - written;
        if (length > DEF_USBD_ENDP3_SIZE - 1)
            length = DEF_USBD_ENDP3_SIZE - 1;
        if (USBFS_Endp_DataUp(DEF_UEP3, buffer + written, length, DEF_UEP_CPY_LOAD) != 0) {
            setWriteError();
            break;
        }
        written += length;
    }
    return written;
}

void USBSerial::flush()
{
    unsigned long start = millis();
    while (USBFS_Endp_Busy[DEF_UEP3] && *this && millis() - start < 250)
        yield();
    if (USBFS_Endp_Busy[DEF_UEP3])
        setWriteError();
}

USBSerial::operator bool()
{
    return started && USBFS_DevEnumStatus && USBFS_LineOpen;
}

#endif
