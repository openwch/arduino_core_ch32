/*
Version made by Matheus Braga Almeida
*/
#include "Wire.hpp"
#include <debug.h>

uint8_t Wire::rxBuffer[BUFFER_LENGTH];
uint8_t Wire::rxBufferIndex = 0;
uint8_t Wire::rxBufferLength = 0;

uint8_t Wire::txAddress = 0;
uint32_t Wire::txClock = 100000;
uint8_t Wire::txBuffer[BUFFER_LENGTH];
uint8_t Wire::txBufferIndex = 0;
uint8_t Wire::txBufferLength = 0;

uint8_t Wire::transmitting = 0;
//void (*Wire::user_onRequest)(void);
//void (*Wire::user_onReceive)(int);

bool Wire::started = false;

void Wire::SetupWire(bool force){
    if (!started || force){
        //Its necessary to setup the GPIOs for the i2c.
        GPIO_InitTypeDef GPIO_InitStructure={0};

        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init( GPIOC, &GPIO_InitStructure );

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init( GPIOC, &GPIO_InitStructure );

        ApplyWireConfig();
    }
}

void Wire::ApplyWireConfig(){
    I2C_InitTypeDef I2C_InitTSturcture={0};
    I2C_InitTSturcture.I2C_ClockSpeed = txClock;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = txAddress;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );
}

Wire::Wire(){
    begin();
}

void Wire::begin(void)
{
    rxBufferIndex = 0;
    rxBufferLength = 0;

    txBufferIndex = 0;
    txBufferLength = 0;

    SetupWire(true);
  /*twi_init();
  twi_attachSlaveTxEvent(onRequestService); // default callback must exist
  twi_attachSlaveRxEvent(onReceiveService); // default callback must exist*/
}

void Wire::begin(uint8_t address)
{
  txAddress = address;
  begin();
}

void Wire::begin(int address)
{
  begin((uint8_t)address);
}

void Wire::end(void)
{
  I2C_DeInit(I2C1);
}

void Wire::setClock(uint32_t clock)
{
  txClock = clock;
  ApplyWireConfig();
}

void Wire::setWireTimeout(uint32_t timeout, bool reset_with_timeout){
  //twi_setTimeoutInMicros(timeout, reset_with_timeout);
}

/***
 * Returns the TWI timeout flag.
 *
 * @return true if timeout has occurred since the flag was last cleared.
 */
bool Wire::getWireTimeoutFlag(void){
  return false;
}

/***
 * Clears the TWI timeout flag.
 */
void Wire::clearWireTimeoutFlag(void){

}



void Wire::beginTransmission(uint8_t addr){
    txAddress = addr;
    transmitting = 1;
    txBufferIndex = 0;
    txBufferLength = 0;
    ApplyWireConfig();
}

void Wire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

uint8_t Wire::endTransmission(uint8_t sendStop)
{
  // transmit buffer (blocking)
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, sendStop);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t Wire::endTransmission(void)
{
  return endTransmission(true);
}



uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{

    if (isize > 0) {
        // send internal address; this mode allows sending a repeated start to access
        // some devices' internal registers. This function is executed by the hardware
        // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

        beginTransmission(address);

        // the maximum size of internal address is 3 bytes
        if (isize > 3){
            isize = 3;
        }

        // write internal register address - most significant byte first
        while (isize-- > 0)
            write((uint8_t)(iaddress >> (isize*8)));
        endTransmission(false);
    }


    if(quantity > BUFFER_LENGTH){
        quantity = BUFFER_LENGTH;
    }

    uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = read;

    return read;
}



uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t Wire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t Wire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t Wire::write(uint8_t data)
{
  if(transmitting){
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= BUFFER_LENGTH){
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t Wire::write(const uint8_t *data, size_t quantity)
{
  if(transmitting){
  // in master transmitter mode
    for(size_t i = 0; i < quantity; ++i){
        if (write(data[i]) == 0){
            return i;
        }
    }
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(data, quantity);
  }
  return quantity;
}

int Wire::available(void)
{
  return rxBufferLength - rxBufferIndex;
}

int Wire::read(void)
{
  int value = -1;

  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

int Wire::peek(void)
{
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void Wire::flush(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;
}

uint8_t Wire::readRegisters(uint8_t address, uint8_t* data, size_t length){
    beginTransmission(txAddress);
    write(address);
    endTransmission(false);

    requestFrom(txAddress, length);

    for (size_t i = 0; i < length; i++) {
        *data++ = read();
    }

    endTransmission(true);

    return length;
}


uint8_t Wire::twi_writeTo(uint8_t txAddress, uint8_t *txBuffer, uint16_t txBufferLength, bool sendStop){


    I2C_GenerateSTART( I2C1, ENABLE );
    int cycles = 1000;
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ){
        cycles--;
        if (cycles <= 0){
            //Make sure to close the i2c
            I2C_GenerateSTOP( I2C1, ENABLE );
            return 0xff;
        }
    }

    I2C_Send7bitAddress( I2C1, txAddress << 1, I2C_Direction_Transmitter );

    cycles = 1000;
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ){
        cycles--;
        if (cycles <= 0){
            //Make sure to close the i2c
            I2C_GenerateSTOP( I2C1, ENABLE );
            return 0xff;
        }
    }

    for (int i=0;i<txBufferLength;i++){
        twi_transmit( &txBuffer[i], 1);
    }

    if (sendStop){
        I2C_GenerateSTOP( I2C1, ENABLE );
    }

    return 0;
}


int Wire::readRegister(uint8_t address)
{
  uint8_t value;

  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }

  return value;
}


int Wire::writeRegister(uint8_t address, uint8_t value){
    beginTransmission(txAddress);
    write(address);
    write(value);
    if (endTransmission() != 0) {
      return 0;
    }
    return 1;
}



bool Wire::twi_transmit(const  uint8_t *bytes, size_t count){
    while(count > 0){
        I2C1->DATAR = *bytes;
        ++bytes;
        --count;
        int cycles = 1000;
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
            cycles--;
            if (cycles <= 0){
                printf("Timed out on: Wire::twi_write\n");
                I2C_GenerateSTOP( I2C1, ENABLE );
                return false;
            }
        }
    }
    return true;
}

uint8_t Wire::twi_readFrom(uint8_t txAddress, uint8_t *txBuffer, uint16_t txBufferLength, bool sendStop){

    int cycles = 1000;

    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ){
        cycles--;
        if (cycles <= 0){
            printf("Timed out on: Wire::twi_readFrom I2C is busy\n");
            I2C_GenerateSTOP( I2C1, ENABLE );
            return false;
        }
    }

    I2C_Send7bitAddress( I2C1, txAddress << 1, I2C_Direction_Receiver  );

    cycles = 1000;
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) ){
        cycles--;
        if (cycles <= 0){
            //Make sure to close the i2c
            printf("Timed out on: Wire::I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED\n");
            I2C_GenerateSTOP( I2C1, ENABLE );
            return uint8_t(0);
        }
    }


    for (int i=0;i<txBufferLength;i++){


        if (i == (txBufferLength-1) ){
            I2C_AcknowledgeConfig( I2C1, DISABLE );
            while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET ){
                cycles--;
                if (cycles <= 0){
                    //Make sure to close the i2c
                    printf("Timed out on: Wire::read last byte\n");
                    I2C_GenerateSTOP( I2C1, ENABLE );
                    return uint8_t(i);
                }
            }
            uint8_t data = I2C1->DATAR;
            rxBuffer[i] = data;
            rxBufferLength++;
        }else{
            cycles = 1000;
            while( !I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE )){
                cycles--;
                if (cycles <= 0){
                    //Make sure to close the i2c
                    printf("Timed out on: Wire::read\n");
                    I2C_GenerateSTOP( I2C1, ENABLE );
                    return uint8_t(i);
                }
            }
            uint8_t data = I2C1->DATAR;
            rxBuffer[i] = data;
            rxBufferLength++;
        }

    }

    if (sendStop){
        I2C_GenerateSTOP( I2C1, ENABLE );
    }

    return rxBufferLength;
}
