#include <usbpd_def.h>
#include <usbpd_sink.h>

#if defined(TIM_MODULE_ENABLED)
    // Initialise HardwareTimer for TIM3 (could also be another timer)
HardwareTimer *USBPD_Timer = new HardwareTimer(TIM3);
#else
    // keep original bypass of CH32 core API and let this library initialize timer3
#endif


// Currently only CH32L103 and CH32X035 have been tested. CH32V205 should also support USBPD
// Most other CH32 don't support USBPD and probably give errors when compiled.
#if defined(CH32L10x)
    #define KEY_INPUT A0
#else
    #define KEY_INPUT A9     //A10=PC0 is not available on QFN20, use A9=PB1=RX4 instead.
#endif

uint8_t myIndex = 0;
Request_voltage_t setVoltage = REQUEST_5v;

#if defined(TIM_MODULE_ENABLED)
// use HarwareTimer callback to handle USBPD things every 5ms
void USBPD_HWTIM_Callback(void)
{
    USBPD_Timer_Callback();
    USBPD_Timer->resume();
}
#else
// if HarwareTimer is not enabled, the USBPD library will use its own Timer3 interrupt handler
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("CH32 USBPD_sink_request_voltage example.");

    // initialize USBPD
    usbpd_sink_init();

#if defined(TIM_MODULE_ENABLED)
    // Attach process to the HardwareTimer instance for Timer 3, could also be set to another timer
    USBPD_Timer->setOverflow(5000, HERTZ_FORMAT); // 5ms
    USBPD_Timer->attachInterrupt(USBPD_HWTIM_Callback);
    USBPD_Timer->resume();
#endif

    pinMode(KEY_INPUT,INPUT_PULLUP); 
}

void loop() {
    // check if set voltage can be set
    if(usbpd_sink_get_ready())
    {
        if(usbpd_sink_set_request_fixed_voltage(setVoltage) == false)
        {
            Serial.printf("unsupported voltage\r\n");
        }
    }

    // when button is pressed, change to a higher voltage, then back to lowest
    if(digitalRead(KEY_INPUT) == 0)
    {
        delay(50);
        if(digitalRead(KEY_INPUT) == 0)
        {
            while(digitalRead(KEY_INPUT) == 0);
            
            myIndex++;
            if(myIndex>4) myIndex = 0;
            Serial.printf("key pressed, setting voltage %d\r\n",myIndex);
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
