#include "CAN.h"

CANClass CAN;

#if defined(CH32V4x7)
  #define CAN_GPIO_SPEED  GPIO_Speed_High
#else
  #define CAN_GPIO_SPEED  GPIO_Speed_50MHz
#endif

int CANClass::setPins(uint32_t rx, uint32_t tx)
{
  PinName r = digitalPinToPinName(rx), t = digitalPinToPinName(tx);
  if (r == PA_11 && t == PA_12) {
    _pins = 0;
  } else if (r == PB_8 && t == PB_9) {
    _pins = 1;
  } else if (r == PD_0 && t == PD_1) {
    _pins = 2;
  } else {
    return 0;
  }
  return 1;
}

static void canPins(uint8_t pins)
{
  static const struct { GPIO_TypeDef *port; uint16_t rx, tx; uint32_t gpioClock; } map[3] = {
    {GPIOA, GPIO_Pin_11, GPIO_Pin_12, RCC_APB2Periph_GPIOA},
    {GPIOB, GPIO_Pin_8,  GPIO_Pin_9,  RCC_APB2Periph_GPIOB},
    {GPIOD, GPIO_Pin_0,  GPIO_Pin_1,  RCC_APB2Periph_GPIOD},
  };
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | map[pins].gpioClock, ENABLE);
  if (pins == 1) {
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
  } else if (pins == 2) {
    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);
  }

  GPIO_InitTypeDef g = {0};
  g.GPIO_Speed = CAN_GPIO_SPEED;
  g.GPIO_Pin = map[pins].tx;
  g.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(map[pins].port, &g);
  g.GPIO_Pin = map[pins].rx;
  g.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(map[pins].port, &g);
}

int CANClass::init(uint8_t mode)
{
  RCC_ClocksTypeDef c;
  RCC_GetClocksFreq(&c);
  // 16 time quanta per bit, sample point at 14/16 = 87.5 %
  if (_baudRate <= 0 || c.PCLK1_Frequency % (_baudRate * 16) != 0) {
    return 0;
  }
  uint32_t prescaler = c.PCLK1_Frequency / (_baudRate * 16);
  if (prescaler == 0 || prescaler > 1024) {
    return 0;
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  CAN_InitTypeDef init = {0};
  init.CAN_TTCM = DISABLE;
  init.CAN_ABOM = ENABLE;       // leave bus-off by itself
  init.CAN_AWUM = DISABLE;
  init.CAN_NART = DISABLE;      // retransmit until acknowledged
  init.CAN_RFLM = DISABLE;
  init.CAN_TXFP = ENABLE;       // send in the order written
  init.CAN_Mode = mode;
  init.CAN_SJW = CAN_SJW_1tq;
  init.CAN_BS1 = CAN_BS1_13tq;
  init.CAN_BS2 = CAN_BS2_2tq;
  init.CAN_Prescaler = prescaler;
  if (CAN_Init(CAN1, &init) != CAN_InitStatus_Success) {
    return 0;
  }
  _started = true;
  return 1;
}

int CANClass::begin(long baudRate)
{
  _baudRate = baudRate;
  canPins(_pins);
  if (!init(CAN_Mode_Normal)) {
    return 0;
  }
  acceptAll();
  return 1;
}

int CANClass::loopback()
{
  if (!_started) {
    return 0;
  }
  return init(CAN_Mode_LoopBack);
}

void CANClass::end()
{
  CAN_DeInit(CAN1);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, DISABLE);
  _started = false;
}

static void setFilter(uint32_t idReg, uint32_t maskReg)
{
  CAN_FilterInitTypeDef f = {0};
  f.CAN_FilterNumber = 0;
  f.CAN_FilterMode = CAN_FilterMode_IdMask;
  f.CAN_FilterScale = CAN_FilterScale_32bit;
  f.CAN_FilterIdHigh = idReg >> 16;
  f.CAN_FilterIdLow = idReg & 0xffff;
  f.CAN_FilterMaskIdHigh = maskReg >> 16;
  f.CAN_FilterMaskIdLow = maskReg & 0xffff;
  f.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  f.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&f);
}

void CANClass::acceptAll()
{
  setFilter(0, 0);
}

// filter register layout: STID[31:21] EXID[20:3] IDE[2] RTR[1]
int CANClass::filter(long id, long mask)
{
  setFilter(((uint32_t)id & 0x7ff) << 21, (((uint32_t)mask & 0x7ff) << 21) | 0x4);
  return 1;
}

int CANClass::filterExtended(long id, long mask)
{
  setFilter((((uint32_t)id & 0x1fffffff) << 3) | 0x4, (((uint32_t)mask & 0x1fffffff) << 3) | 0x4);
  return 1;
}

int CANClass::beginPacket(long id, int dlc, bool rtr)
{
  if (id < 0 || id > 0x7ff || dlc > 8) {
    return 0;
  }
  _txStarted = true;
  _txId = id;
  _txExtended = false;
  _txRtr = rtr;
  _txDlc = dlc;
  _txLength = 0;
  return 1;
}

int CANClass::beginExtendedPacket(long id, int dlc, bool rtr)
{
  if (id < 0 || id > 0x1fffffff || dlc > 8) {
    return 0;
  }
  _txStarted = true;
  _txId = id;
  _txExtended = true;
  _txRtr = rtr;
  _txDlc = dlc;
  _txLength = 0;
  return 1;
}

size_t CANClass::write(uint8_t byte)
{
  return write(&byte, 1);
}

size_t CANClass::write(const uint8_t *buffer, size_t size)
{
  if (!_txStarted) {
    return 0;
  }
  if (size > (size_t)(8 - _txLength)) {
    size = 8 - _txLength;
  }
  memcpy(&_txData[_txLength], buffer, size);
  _txLength += size;
  return size;
}

int CANClass::endPacket()
{
  if (!_txStarted || !_started) {
    return 0;
  }
  _txStarted = false;

  CanTxMsg m = {0};
  if (_txExtended) {
    m.ExtId = _txId;
    m.IDE = CAN_Id_Extended;
  } else {
    m.StdId = _txId;
    m.IDE = CAN_Id_Standard;
  }
  m.RTR = _txRtr ? CAN_RTR_Remote : CAN_RTR_Data;
  m.DLC = _txDlc >= 0 ? _txDlc : _txLength;
  memcpy(m.Data, _txData, _txLength);

  uint8_t box = CAN_Transmit(CAN1, &m);
  if (box == CAN_TxStatus_NoMailBox) {
    return 0;
  }
  uint32_t t0 = millis();
  while (CAN_TransmitStatus(CAN1, box) != CAN_TxStatus_Ok) {
    if (millis() - t0 > 100) {
      CAN_CancelTransmit(CAN1, box);   // nobody acknowledged it
      return 0;
    }
  }
  return 1;
}

int CANClass::parsePacket()
{
  if (!_started || CAN_MessagePending(CAN1, CAN_FIFO0) == 0) {
    return 0;
  }
  CanRxMsg m;
  CAN_Receive(CAN1, CAN_FIFO0, &m);
  _rxExtended = m.IDE == CAN_Id_Extended;
  _rxId = _rxExtended ? (long)m.ExtId : (long)m.StdId;
  _rxRtr = m.RTR == CAN_RTR_Remote;
  _rxDlc = m.DLC;
  _rxLength = _rxRtr ? 0 : (m.DLC > 8 ? 8 : m.DLC);
  _rxIndex = 0;
  memcpy(_rxData, m.Data, _rxLength);
  // a remote frame has no data; report its DLC so the caller still sees a packet
  return _rxRtr ? (_rxDlc ? _rxDlc : 1) : (_rxLength ? _rxLength : 1);
}

int CANClass::available()
{
  return _rxLength - _rxIndex;
}

int CANClass::read()
{
  return available() ? _rxData[_rxIndex++] : -1;
}

int CANClass::peek()
{
  return available() ? _rxData[_rxIndex] : -1;
}
