#include <usbpd_def.h>
#include <usbpd_sink.h>

#define KEY_INPUT  A10     //PC0

uint8_t myIndex = 0;
uint8_t setVoltage = REQUEST_5v;

void setup() {
  // put your setup code here, to run once:

    Serial.begin(115200);
    usbpd_sink_init();

    pinMode(KEY_INPUT,INPUT_PULLUP); 
}

void loop() {
  // put your main code here, to run repeatedly:

    if(usbpd_sink_get_ready())
    {
        usbpd_sink_set_request_fixed_voltage(setVoltage);
    }

// button, myIndex++
    if(digitalRead(KEY_INPUT) == 0)
    {
        delay(50);
        if(digitalRead(KEY_INPUT) == 0)
        {
            while(digitalRead(KEY_INPUT) == 0);
            
            myIndex++;
            if(myIndex>4) myIndex = 0;
            Serial.printf("key press %d\r\n",myIndex);
        }
    } 
    switch(myIndex)
    {
        case 0:
            setVoltage = REQUEST_5v;
            break;
        case 1:
            setVoltage = REQUEST_9v;
            break;
        case 2:
            setVoltage = REQUEST_12v;
            break;
        case 3:
            setVoltage = REQUEST_15v;
            break;
        case 4:
            setVoltage = REQUEST_20v;
            break;
        default:
            setVoltage = REQUEST_5v;
            break;
    }

}
