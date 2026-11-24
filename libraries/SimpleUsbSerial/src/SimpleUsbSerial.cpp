#include <Arduino.h>
#include "SimpleUsbSerial.h"

extern "C" {
#if defined(CH585) || defined(CH32V30x)
    uint16_t USBSerial_available();
#else
    uint8_t USBSerial_available();
#endif
    char USBSerial_read();
    void USBSerial_flush(void);
    uint8_t USBSerial_write( char c);
    bool USBSerial_bool();
}

void SerialOverUsbCdc::begin(unsigned long baud, byte config){
    USBInit();
}

int SerialOverUsbCdc::available(void)
{
  return USBSerial_available();
}
int SerialOverUsbCdc::peek(void)
{
  return -1; // not implemented
}
int SerialOverUsbCdc::read(void)
{
  return USBSerial_read();
}

size_t SerialOverUsbCdc::write(uint8_t c)
{
  return USBSerial_write(c);
}

void SerialOverUsbCdc::flush(void)
{
  USBSerial_flush();
}

SerialOverUsbCdc::operator bool() {
	return USBSerial_bool();
}

SerialOverUsbCdc SerialUSB;