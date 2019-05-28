extern "C" {
	#include "stm32l4xx_hal.h"
}
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>

class LT8920{
public:
	enum DataRate {
	  LT8920_1MBPS,     /** default transmit rate */
	  LT8920_250KBPS,   /** 250 Kpbs, only on lt8910 */
	  LT8920_125KBPS,   /** 125 Kbps, only on lt8910 */
	  LT8920_62KBPS,     /** 62 Kbps, only on lt8910 */
	  LT8920_INVALID
	};
	LT8920(SPI_HandleTypeDef* spi);
	uint8_t writeRegister2(uint8_t reg, uint8_t high, uint8_t low);
	inline uint8_t writeRegister(uint8_t reg, uint16_t val){
		return writeRegister2(reg, val >> 8, val);
	}
	uint8_t writeRegisters(uint8_t start_reg, uint16_t* values, size_t regs);
	uint16_t readRegister(uint8_t reg);
	void begin();
    void setChannel(uint8_t channel);
    uint8_t getChannel();
    void setCurrentControl(uint8_t power, uint8_t gain);
    bool setDataRate(DataRate rate);
    DataRate getDataRate();
    void sleep();
	int read(uint8_t *buffer, size_t maxBuffer);
	void startListening();
	void stopListening();
	void setClock(uint8_t clock);
	bool sendPacket(uint8_t *data, size_t packetSize);
	void setSyncWord(uint64_t syncWord);
	void setSyncWordLength(uint8_t length);
	uint8_t getRSSI();

	SPI_HandleTypeDef* _spi;
	uint8_t _channel;
};
