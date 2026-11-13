#ifndef CH32X035_USB_SERIAL_H
#define CH32X035_USB_SERIAL_H

#include "Stream.h"

class USBSerial : public Stream {
public:
    void begin(unsigned long baud);
    void begin(unsigned long baud, uint8_t config) { (void)config; begin(baud); }
    void end();
    int available() override;
    int peek() override;
    int read() override;
    size_t write(uint8_t value) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    void flush() override;
    using Print::write;
    operator bool();
};

extern USBSerial SerialUSB;

#endif
