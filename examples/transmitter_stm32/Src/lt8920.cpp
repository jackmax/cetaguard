#include "LT8920.h"
#include "main.h"

#define REGISTER_READ       0b10000000
#define REGISTER_WRITE      0b00000000
#define REGISTER_MASK       0b01111111

#define R_CHANNEL           7
#define CHANNEL_RX_BIT      7
#define CHANNEL_TX_BIT      8
#define CHANNEL_MASK        0b01111111
#define DEFAULT_CHANNEL     0x31

#define R_CURRENT           9
#define CURRENT_POWER_SHIFT 12
#define CURRENT_POWER_MASK  0b1111000000000000
#define CURRENT_GAIN_SHIFT  7
#define CURRENT_GAIN_MASK   0b0000011110000000

#define R_DATARATE          44
#define DATARATE_MASK       0xFF00
#define DATARATE_1MBPS      0x0100
#define DATARATE_250KBPS    0x0400
#define DATARATE_125KBPS    0x0800
#define DATARATE_62KBPS     0x1000

#define R_SYNCWORD1         36
#define R_SYNCWORD2         37
#define R_SYNCWORD3         38
#define R_SYNCWORD4         39

#define R_PACKETCONFIG      41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400

#define R_STATUS            48
#define STATUS_CRC_BIT      15

#define R_FIFO              50
#define R_FIFO_CONTROL      52

#define _BV(x) (1 << x)

LT8920::LT8920(SPI_HandleTypeDef* spi){
	//HAL_SPI_TransmitReceive_DMA(&hspi1, message_buffer, recv_buffer, 64);
	_spi = spi;
	_channel = DEFAULT_CHANNEL;
}

uint8_t LT8920::writeRegister2(uint8_t reg, uint8_t high, uint8_t low){
  WRITE_PIN_L(SPI1_NCS, 0);

  uint8_t tx[3];
  tx[0] = REGISTER_WRITE | (REGISTER_MASK & reg);
  tx[1] = high;
  tx[2] = low;
  uint8_t rx[3];

  HAL_SPI_TransmitReceive(_spi, tx, rx, 3, 1);

  WRITE_PIN_L(SPI1_NCS, 1);
  return rx[0];
}

uint8_t LT8920::writeRegisters(uint8_t start_reg, uint16_t* values, size_t regs){
	WRITE_PIN_L(SPI1_NCS, 0);

	uint8_t tx = REGISTER_WRITE | (REGISTER_MASK & start_reg);
	uint8_t rx;
	for (uint i = 0; i < regs; i++){
		values[i] = (values[i] << 8) | (values[i] >> 8);
	}

	HAL_SPI_TransmitReceive(_spi, &tx, &rx, 1, 1);
	HAL_SPI_Transmit(_spi, (uint8_t*) values, regs*2, 1);

	WRITE_PIN_L(SPI1_NCS, 1);
	return rx;
}

uint16_t LT8920::readRegister(uint8_t reg){
  WRITE_PIN_L(SPI1_NCS, 0);

  uint8_t tx[3];
  tx[0] = REGISTER_READ | (REGISTER_MASK & reg);
  uint8_t rx[3];

  HAL_SPI_TransmitReceive(_spi, tx, rx, 3, 1);

  WRITE_PIN_L(SPI1_NCS, 1);
  return ((uint16_t)rx[1] << 8) | ((uint16_t)rx[2] & 0xFF);
}
/*
*/
void LT8920::begin(){
	//WRITE_PIN_L(Radio_RST, 0);
	//HAL_Delay(200);
	//WRITE_PIN_L(Radio_RST, 1);
	//HAL_Delay(200);

	//setup
	writeRegister(0, 0x6fe0);
	writeRegister(1, 0x5681);
	writeRegister(2, 0x6617);
	writeRegister(4, 0x9cc9);    //why does this differ from powerup (5447)
	writeRegister(5, 0x6637);    //why does this differ from powerup (f000)
	writeRegister(R_CHANNEL, _channel);  // Frequency = 2402 + channel
	writeRegister(8, 0x6c90);    //power (default 71af) UNDOCUMENTED
	writeRegister(R_CURRENT, 0x4800);

	setCurrentControl(4, 0);     // power & gain.

	writeRegister(10, 0x7ffd);   //bit 0: XTAL OSC enable
	writeRegister(11, 0x0000);   //bit 8: Power down RSSI (0=  RSSI operates normal)
	writeRegister(12, 0x0000);
	writeRegister(13, 0x48bd);   //(default 4855)

	uint16_t regs_22_28[7] = {
			0x00ff,
			0x8005,  //bit 2: Calibrate VCO before each Rx/Tx enable
			0x0067,
			0x1659,
			0x19e0,
			0x1300,  //bits 5:0, Crystal Frequency adjust
			0x1800,
	};
	writeRegisters(22, regs_22_28, 7);

	uint16_t regs_32_45[] = {
	/* 32 */0x5000,  //AAABBCCCDDEEFFFG  A preamble length, B, syncword length, c trailer length, d packet type, E FEC_type, F BRCLK_SEL, G reserved
					 //0x5000 = 0101 0000 0000 0000 = preamble 010 (3 bytes), B 10 (48 bits)
	/* 33 */0x3fc7,
	/* 34 */0x2000,
	/* 35 */0x0300,  //POWER mode,  bit 8/9 on = retransmit = 3x (default)
	/* 36 */0xe621,	 //syncword lowest 16 bits
	/* 37 */0x0380,
	/* 38 */0x5a5a,
	/* 39 */0x0380,	 //syncword highest 16 bits
	/* 40 */0x4401,  //max allowed error bits = 0 (01 = 0 error bits)
	/* 41 */PACKETCONFIG_CRC_ON | PACKETCONFIG_PACK_LEN_ENABLE | PACKETCONFIG_FW_TERM_TX /*| PACKETCONFIG_AUTO_ACK*/,
	/* 42 */0xfdb0,
	/* 43 */0x000f,
	/* 44 */0x0100,	 //data rate
	/* 45 */0x0080,
	};
	writeRegisters(32, regs_32_45, 14);

	writeRegister(R_FIFO, 0);
	readRegister(R_FIFO);

	writeRegister(R_FIFO_CONTROL, 0x8080); //Fifo Rx/Tx queue reset
	writeRegister(R_CHANNEL, _BV(CHANNEL_TX_BIT));  //set TX mode.  (TX = bit 8, RX = bit 7, so RX would be 0x0080)
	HAL_Delay(2);
}


void LT8920::setChannel(uint8_t channel)
{
  _channel = channel;
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK));
}

uint8_t LT8920::getChannel()
{
  return _channel;
}

void LT8920::setCurrentControl(uint8_t power, uint8_t gain)
{
  writeRegister(R_CURRENT,
                ((power << CURRENT_POWER_SHIFT) & CURRENT_POWER_MASK) |
                ((gain << CURRENT_GAIN_SHIFT) & CURRENT_GAIN_MASK));
}

bool LT8920::setDataRate(DataRate rate)
{
  uint16_t newValue;
  uint16_t reg45Value = 0x0080;

  switch (rate)
  {
    case LT8920_1MBPS:
      newValue = DATARATE_1MBPS;
      reg45Value = 0x0080;
      break;
    case LT8920_250KBPS:
      newValue = DATARATE_250KBPS;
      reg45Value = 0x0552;
      break;
    case LT8920_125KBPS:
      newValue = DATARATE_125KBPS;
      reg45Value = 0x0552;
      break;
    case LT8920_62KBPS:
      newValue = DATARATE_62KBPS;
      reg45Value = 0x0552;
      break;
    default:
      return false;
  }

  writeRegister(R_DATARATE, newValue);
  writeRegister(45, reg45Value);
  return ( (readRegister(R_DATARATE) & DATARATE_MASK) == (newValue & DATARATE_MASK));
}

LT8920::DataRate LT8920::getDataRate()
{
  uint16_t value = readRegister(R_DATARATE) & DATARATE_MASK;
  switch (value)
  {
    case DATARATE_1MBPS:
      return LT8920_1MBPS;
    case DATARATE_250KBPS:
      return LT8920_250KBPS;
    case DATARATE_125KBPS:
      return LT8920_125KBPS;
    case DATARATE_62KBPS:
      return LT8920_62KBPS;
  }
  return LT8920_INVALID;
}

void LT8920::sleep()
{
  //set bit 14 on register 35.
  writeRegister(35, readRegister(35) | _BV(14));
}

void LT8920::startListening()
{
  writeRegister(R_FIFO_CONTROL, 0x8080);  //flush rx
  HAL_Delay(1);
  writeRegister(R_CHANNEL, _channel & CHANNEL_MASK);   //turn off rx/tx
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_RX_BIT));   //enable RX
  HAL_Delay(1);
}

void LT8920::stopListening()
{
  writeRegister(R_CHANNEL, _channel & CHANNEL_MASK);   //turn off rx/tx
  HAL_Delay(1);
}

/* set the BRCLK_SEL value */
void LT8920::setClock(uint8_t clock)
{
  //register 32, bits 3:1.
  uint16_t val = readRegister(35);
  val &= 0b1111111111110001;
  val |= ((clock & 0x07) << 1);;

  writeRegister(35, val);
}

bool LT8920::sendPacket(uint8_t *data, size_t packetSize){
  if (packetSize < 1 || packetSize > 255) {
    return false;
  }

  writeRegister(R_CHANNEL, 0x0000);
  writeRegister(R_FIFO_CONTROL, 0x8000);  //flush tx

  //packets are sent in 16bit words, and the first word will be the packet size.
  //start spitting out words until we are done.

  WRITE_PIN_L(SPI1_NCS, 0);
  uint8_t setup[2] = { REGISTER_WRITE | (REGISTER_MASK & R_FIFO), (uint8_t) packetSize }; //the cast is there to shut up the compiler
  HAL_SPI_Transmit(_spi, setup, 2, 1);
  HAL_SPI_Transmit(_spi, data, packetSize, 10);
  WRITE_PIN_L(SPI1_NCS, 1);
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_TX_BIT));   //enable TX

  return true;
}

int LT8920::read(uint8_t *buffer, size_t maxBuffer){
  uint16_t value = readRegister(R_STATUS);
  if (value & STATUS_CRC_BIT){
	//CRC error
	return -1;
  }
  else{
    //CRC ok
	WRITE_PIN_L(SPI1_NCS, 0);
	uint8_t tx = REGISTER_READ | (REGISTER_MASK & R_FIFO);
	uint8_t packetSize;
	HAL_SPI_Transmit(_spi, &tx, 1, 1);
	HAL_SPI_Receive(_spi, &packetSize, 1, 1);

	if(maxBuffer < packetSize)
	{
		//BUFFER TOO SMALL
		return -2;
	}

	HAL_SPI_Receive(_spi, buffer, packetSize, 10);
	WRITE_PIN_L(SPI1_NCS, 1);

	return packetSize;
  }
}

void LT8920::setSyncWord(uint64_t syncWord){
  writeRegister(R_SYNCWORD1, syncWord);
  writeRegister(R_SYNCWORD2, syncWord >> 16);
  writeRegister(R_SYNCWORD3, syncWord >> 32);
  writeRegister(R_SYNCWORD4, syncWord >> 48);
}

void LT8920::setSyncWordLength(uint8_t option){
  option &= 0x03;
  option <<= 11;

  writeRegister(32, (readRegister(32) & 0b0001100000000000) | option);
}

uint8_t LT8920::getRSSI(){
    //RSSI: 15:10
    uint16_t value = readRegister(6);

    return (value >> 10);
}
