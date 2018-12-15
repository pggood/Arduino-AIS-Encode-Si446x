#ifndef ais_h
#define ais_h

#include "Arduino.h"
class Ais
{
  public:
    Ais();
    void AISMessage18encode();
    char lat[8];
    char lon[9];
    bool north;
    bool east;
    volatile uint32_t sog;
    volatile uint32_t cog;
    volatile uint32_t utc;
    volatile uint32_t MMSI;
    volatile uint8_t TXPacket[33];
    volatile uint8_t Packet[33];
  private:
  volatile int32_t latitude;
  volatile int32_t longitude;
  void addBits(uint8_t *bitVector, uint16_t &size, int32_t value, uint8_t numBits);
  void putBits(uint8_t *bitVector, uint32_t value, uint8_t numBits);
  void reverseEachByte(uint8_t *bitVector, uint16_t size);
  void addString(uint8_t *bitVector, uint16_t &size, String &value, uint8_t maxChars);
  void payloadToBytes(uint8_t *bitVector, uint16_t numBits, uint8_t *byteArray);
  uint16_t reverseBits(uint16_t crc);
  uint16_t crc16(uint8_t *bytes, uint16_t length);
  void finalize(uint8_t *payload, uint16_t &size);
  void bitStuff(uint8_t *buff, uint16_t &size);
  void constructHDLCFrame(uint8_t *buff, uint16_t &size);
  void nrziEncode(uint8_t *buff, uint16_t &size);
  uint32_t coordinateToUINT32(double value);
  uint16_t mSize;
  uint16_t mPosition;
};
#endif
