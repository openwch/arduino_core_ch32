#ifndef __CH573_USB_SERIAL_H__
#define __CH573_USB_SERIAL_H__

#include "SimpleUsbCdc.h"

class SerialOverUsbCdc : public Stream {
public:

    void begin(unsigned long baud)
    {
      begin(baud, SERIAL_8N1);     //SERIAL_9E1_5  SERIAL_8N1
    }
    void begin(){
        begin(115200, SERIAL_8N1);
    }
    void begin(unsigned long, uint8_t);

    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual size_t write(uint8_t);
    virtual void flush(void);
    virtual operator bool();

};

extern SerialOverUsbCdc SerialUSB;

#endif // __CH573_USB_SERIAL_H__
