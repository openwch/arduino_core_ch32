#include <SimpleUsbSerial.h>

__attribute__((section(".highcode")))
#if defined(CH573)
void tx_on_PA12_main(char c) {
    //R32_PA_OUT located at 0x400010a8
    //high 20 bits are 0x40001, low 12 bits are 0x0a8(offset)

    //There is no processor flags register in the RISC-V ISA

    //CH573 used RV32 IMAC, the compressed 16-bit instruction actually creates pipeline timing issue, avoid using them in this function
    //CH573: sw uses 2 clk, add a 32bit nop (add zero,zero,zero) for 8clks per bit

    uint32_t irq_status;

    //SYS_DisableAllIrq( &irq_status );
    irq_status = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    asm volatile(
            "lui   t0, 0x40001                  \n\t"
            "lw    t1, 0x0a8(t0)                \n\t"  //copy R32_PA_OUT to register t1
            //"addi  t2, zero, (1<<4)             \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "lui   t2, (1<<(12-12))               \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "not   t3, t2                       \n\t"  //constant for and logic to set pin low.
            "addi  t5, zero, 12                  \n\t"  //use for sll. slli may be compressed to 16 bit and cause pipeline delay

            "and   t1, t1, t3                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<0)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<1)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<2)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<3)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<4)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<5)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<6)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<7)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //useless
            "andi  t4, %[OUT_CHAR], (1<<8)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //useless
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t2                   \n\t"  //set high for stop bit
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            :
            :[OUT_CHAR]"r"(c)
            :"t0", "t1", "t2", "t3", "t4", "t5"
            );

    //SYS_RecoverIrq( irq_status );
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}
#elif defined(CH32X035)
void tx_on_PA4_main(char c) {
    //R32_PA_OUT located at 0x4001080C
    //high 20 bits are 0x40010, low 12 bits are 0x80C(offset)

    //There is no processor flags register in the RISC-V ISA

    //CH573 used RV32 IMAC, the compressed 16-bit instruction actually creates pipeline timing issue, avoid using them in this function
    //CH573: sw uses 2 clk, add a 32bit nop (add zero,zero,zero) for 8clks per bit

    uint32_t irq_status;

    //SYS_DisableAllIrq( &irq_status );
    // irq_status = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    // PFIC->IRER[0] = 0xffffffff;
    // PFIC->IRER[1] = 0xffffffff;

    asm volatile(
            "lui   t0, 0x40010                  \n\t"
            // 0x80C (2060) exceeds addi's 12-bit signed immediate range (-2048..+2047)
            // split into two addi instructions: 0x7FF (2047) + 13 = 2060
            "addi  t0, t0, 0x7ff                 \n\t"
            "addi  t0, t0, 13                    \n\t"

            "lw    t1, 0x0(t0)                \n\t"  //copy R32_PA_OUT to register t1
            "addi  t2, zero, (1<<4)             \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            //"lui   t2, (1<<(12-12))               \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "not   t3, t2                       \n\t"  //constant for and logic to set pin low.
            "addi  t5, zero, 4                  \n\t"  //use for sll. slli may be compressed to 16 bit and cause pipeline delay

            "and   t1, t1, t3                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<0)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<1)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<2)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<3)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<4)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<5)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<6)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<7)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //useless
            "andi  t4, %[OUT_CHAR], (1<<8)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //useless
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t2                   \n\t"  //set high for stop bit
            "sw    t1, 0x0(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            :
            :[OUT_CHAR]"r"(c)
            :"t0", "t1", "t2", "t3", "t4", "t5"
            );

    //SYS_RecoverIrq( irq_status );
    // PFIC->IENR[0] = (irq_status << 8);
    // PFIC->IENR[1] = (irq_status >> 24);
}
#endif


void setup() {
  #if defined(CH573)
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, HIGH);
  #elif defined(CH32X035)
  pinMode(PA_4, OUTPUT);
  digitalWrite(PA_4, HIGH);
  #endif
  asm("nop");
  delay(1);

  SerialUSB.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1);

  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");


  
  int incoming = SerialUSB.available();
  if (incoming>0){
    for (int i = 0; i < incoming; i++) {
      char c = SerialUSB.read();
      #if defined(CH573)
      tx_on_PA12_main(c);
      #elif defined(CH32X035)
      tx_on_PA4_main(c);
      #endif
    }
    SerialUSB.println("OK!");
    SerialUSB.flush();
  }
  
  
}
