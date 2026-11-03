#include "USBconstant.h"


//Device descriptor
const uint8_t DevDesc[] = {
    0x12,0x01,
#if defined(CH585) || defined(CH32V30x)
    0x00,0x02,  //USB spec release number in BCD
#else
    0x10,0x01,  //USB spec release number in BCD format, USB1.1 (0x10, 0x01).
#endif
    0xEF,0x02,0x01, //bDeviceClass, bDeviceSubClass, bDeviceProtocol 
    DEFAULT_ENDP0_SIZE, //bMaxPacketSize0
    0x09,0x12,0x50,0xC5, // VID PID 
    0x01,0x01,  //version
    0x01,0x02,0x03, //bString
    0x01    //bNumConfigurations
};

const uint16_t DevDescLen = sizeof(DevDesc);

#define CfgDesc_SIZE_For_GCC 75

const uint8_t CfgDesc[] ={
    0x09,0x02,CfgDesc_SIZE_For_GCC & 0xff,CfgDesc_SIZE_For_GCC >> 8,
    0x02,0x01,0x00,0x80,0x64,             //Configuration descriptor (2 interfaces)
    // Interface Association Descriptor, IAD, this packes following 2 interfaces into 1
    0x08,0x0B,0x00,0x02,0x02,0x02,0x01,0x04,
    // Interface 1 (CDC) descriptor
    0x09,0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x04,    // CDC control description, 1 endpoint
    // Functional Descriptor refer to usbcdc11.pdf
    0x05,0x24,0x00,0x10,0x01,                                 //Header Functional Descriptor
    0x05,0x24,0x01,0x00,0x00,                                 //Call Management Functional Descriptor
    0x04,0x24,0x02,0x02,                                      //Direct Line Management Functional Descriptor, Support: Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, Serial_State 
    0x05,0x24,0x06,0x00,0x01,                                 //Union Functional Descriptor, Communication class interface 0, Data Class Interface 1
    0x07,0x05,0x81,0x03,0x08,0x00,0x40,                       //EndPoint descriptor (CDC Upload, Interrupt)
    // Interface 2 (Data Interface) descriptor
    0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x04,             //Data Class Interface descriptor
    0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //endpoint descriptor
    0x07,0x05,0x82,0x02,0x40,0x00,0x00,                       //endpoint descriptor
};

const uint16_t CfgDescLen = sizeof(CfgDesc);

#if defined(CH585) || defined(CH32V30x)
const uint8_t CfgHsDesc[] ={
    0x09,0x02,CfgDesc_SIZE_For_GCC & 0xff,CfgDesc_SIZE_For_GCC >> 8,
    0x02,0x01,0x00,0x80,0x64,             //Configuration descriptor (2 interfaces)
    // Interface Association Descriptor, IAD, this packes following 2 interfaces into 1
    0x08,0x0B,0x00,0x02,0x02,0x02,0x01,0x04,
    // Interface 1 (CDC) descriptor
    0x09,0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x04,    // CDC control description, 1 endpoint
    // Functional Descriptor refer to usbcdc11.pdf
    0x05,0x24,0x00,0x10,0x01,                                 //Header Functional Descriptor
    0x05,0x24,0x01,0x00,0x00,                                 //Call Management Functional Descriptor
    0x04,0x24,0x02,0x02,                                      //Direct Line Management Functional Descriptor, Support: Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, Serial_State 
    0x05,0x24,0x06,0x00,0x01,                                 //Union Functional Descriptor, Communication class interface 0, Data Class Interface 1
    0x07,0x05,0x81,0x03,0x08,0x00,0x40,                       //EndPoint descriptor (CDC Upload, Interrupt)
    // Interface 2 (Data Interface) descriptor
    0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x04,             //Data Class Interface descriptor
    0x07,0x05,0x02,0x02,0x00,0x02,0x00,                       //endpoint descriptor, 512 bytes for high speed
    0x07,0x05,0x82,0x02,0x00,0x02,0x00,                       //endpoint descriptor, 512 bytes for high speed
};

const uint16_t CfgHsDescLen = sizeof(CfgHsDesc);

const uint8_t QualDesc[] = {
    0x0A,0x06,0x00,0x02,0xEF,0x02,0x01,0x40,0x01,0x00
};
const uint16_t QualDescLen = sizeof(QualDesc);

uint8_t CfgOtherDesc[sizeof(CfgHsDesc)] = {0x09, 0x07};
#endif

//String Descriptors
const uint8_t LangDes[]={0x04,0x03,0x09,0x04};           //Language Descriptor
const uint16_t LangDesLen = sizeof(LangDes);
const uint16_t SerDes[]={                                 //Serial String Descriptor
#if defined(CH573) || defined(CH572)
    0x030C,
    'C','H','5','7','x',
#elif defined(CH585)
    0x030C,
    'C','H','5','8','x',
#elif defined(CH32X035)
    0x0312,
    'C','H','3','2','X','0','3','5',
#elif defined(CH32V30x)
    0x0312,
    'C','H','3','2','V','3','0','x',
#endif
};
const uint16_t SerDesLen = sizeof(SerDes);
const uint16_t Prod_Des[]={                                //Produce String Descriptor
#if defined(CH573) || defined(CH572)
    0x0316,
    'C','H','5','7','x','d','u','i','n','o',
#elif defined(CH585)
    0x0316,
    'C','H','5','8','x','d','u','i','n','o',
#elif defined(CH32X035)
    0x031C,
    'C','H','3','2','X','0','3','5','d','u','i','n','o',
#elif defined(CH32V30x)
    0x031C,
    'C','H','3','2','V','3','0','x','d','u','i','n','o',
#endif
};
const uint16_t Prod_DesLen = sizeof(Prod_Des);

const uint16_t CDC_Des[]={
    0x0316,
    'C','D','C',' ','S','e','r','i','a','l',
};
const uint16_t CDC_DesLen = sizeof(CDC_Des);

const uint16_t Manuf_Des[]={    //SDCC is little endian
    0x030E,
    'D','e','q','i','n','g',
};
const uint16_t Manuf_DesLen = sizeof(Manuf_Des);
