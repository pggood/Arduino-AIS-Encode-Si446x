
#include "ais.h"
#include <assert.h>
#define BIT_IS_SET(i, bits)  (1 << i & bits)
#define MAX_AIS_TX_PACKET_SIZE       256
#define swap(a, b) { uint8_t t = a; a = b; b = t; }
#define DEFAULT_COMM_STATE 0b1100000000000000110
//uint8_t mPacket[MAX_AIS_TX_PACKET_SIZE/8+1];
/*static const uint16_t CRC16_XMODEM_TABLE[] =
    { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
            0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231,
            0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339,
            0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462,
            0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
            0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653,
            0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b,
            0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4,
            0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc,
            0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
            0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd,
            0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6,
            0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae,
            0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
            0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f,
            0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188,
            0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080,
            0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9,
            0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
            0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea,
            0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2,
            0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db,
            0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
            0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c,
            0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844,
            0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d,
            0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75,
            0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
            0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26,
            0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f,
            0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17,
            0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
*/            
Ais::Ais(){}            
void Ais::addBits(uint8_t *bitVector, uint16_t &size, int32_t value, uint8_t numBits)
{
    //assert(numBits > 0  && numBits <= 32);
    uint16_t pos = size;
    for ( uint8_t bit = 0; bit < numBits; ++bit, value >>= 1 )  {
        bitVector[pos + numBits-bit-1] = value & 1;
    }

    size += numBits;
}

void Ais::putBits(uint8_t *bitVector, uint32_t value, uint8_t numBits)
{
    // This is used for HDLC framing
    uint16_t pos = 0;
    for ( uint8_t bit = 0; bit < numBits; ++bit, value >>= 1 )  {
        bitVector[pos++] = value & 0x01;
    }
}

void Ais::reverseEachByte(uint8_t *bitVector, uint16_t size)
{
    assert(size % 8 == 0);
    for ( uint16_t i = 0; i < size; i += 8 ) {
        for ( uint8_t j = 0; j < 4; ++j ) {
            swap(bitVector[i+j], bitVector[i+7-j]);
        }
    }
}

void Ais::addString(uint8_t *bitVector, uint16_t &size, String &value, uint8_t maxChars)
{
    assert(value.length() <= maxChars);
    assert(maxChars < 30); // There should be no application for such long strings here
    char s[30];
    memset(s, 0, sizeof s);
    strncpy(s, value.c_str(), value.length());

    uint8_t buffer[32];
    for ( uint8_t c = 0; c < maxChars; ++c ) {
        uint8_t byte = s[c] >= 64 ? s[c]-64 : s[c];
        buffer[c] = byte;
    }

    for ( uint8_t c = 0; c < maxChars; ++c )
        addBits(bitVector, size, buffer[c], 6);
}

void Ais::payloadToBytes(uint8_t *bitVector, uint16_t numBits, uint8_t *byteArray)
{
    for ( uint16_t i = 0; i < numBits; i += 8 ) {
        uint8_t byte = 0;
        for ( uint8_t b = 0; b < 8; ++b ) {
            byte |= (bitVector[i+b] << b);
        }
        byteArray[i/8] = byte;
    }
}
uint16_t Ais::reverseBits(uint16_t crc)
{
    uint16_t result = 0;
    for ( size_t i = 0; i < 16; ++i ) {
        result <<= 1;
        result |= ((crc & (1 << i)) >> i);
    }

    return result;
}
/*uint16_t Ais::crc16(uint8_t *bytes, uint16_t length)
{
    uint16_t crc = 0xffff;
    for ( uint16_t b = 0; b < length; ++b ) {
        crc = ((crc << 8) & 0xff00) ^ CRC16_XMODEM_TABLE[((crc >> 8) & 0xff) ^ bytes[b]];
    }
    return reverseBits(~crc);
}
*/
uint16_t Ais::crc16(uint8_t *bytes, uint16_t length)
{
    uint16_t crc = 0xffff;
        while (length--)
        {
            crc = (crc >> 8) | (crc << 8);
            crc ^= *bytes++;
            crc ^= ((unsigned char) crc) >> 4;
            crc ^= crc << 12;
            crc ^= (crc & 0xFF) << 5;
        }
    return reverseBits(~crc);
}    
void Ais::finalize(uint8_t *payload, uint16_t &size)
{
    // Nothing we send exceeds 256 bits, including preambles and such. 40 bytes is more than enough.
    uint8_t bytes[40];

    // CRC-CCITT calculation
    payloadToBytes(payload, size, bytes);
    uint16_t crc = crc16(bytes, size/8);
    uint8_t crcL = crc & 0x00ff;
    uint8_t crcH = (crc & 0xff00) >> 8; 
    addBits(payload, size, crcL, 8);
    
    addBits(payload, size, crcH, 8);
   
    payloadToBytes(payload, size, bytes);

    // Encoding for transmission
    reverseEachByte(payload, size);
    bitStuff(payload, size);
    constructHDLCFrame(payload, size);
    nrziEncode(payload, size);
    int mSize     = 0;
    uint16_t index ;
    uint8_t offset;
    for ( uint16_t i = 0; i < size; ++i ) {
        index = mSize / 8;
        offset = mSize % 8;
        if (payload[i]) Packet[index] |= ( 1 << offset );        
        ++mSize; 
    }
    for ( uint16_t i = 0; i < mSize/8; ++i ) {     
          Serial.print(Packet[i],HEX);
          Serial.print(" ");
        }
        Serial.println();
    
}

void Ais::bitStuff(uint8_t *buff, uint16_t &size)
{
    uint16_t numOnes = 0;
    for ( uint16_t i = 0; i < size; ++i ) {
        switch(buff[i]) {
            case 0:
                numOnes = 0;
                break;
            case 1:
                ++numOnes;
                if ( numOnes == 5 ) {
                    // Insert a 0 right after this one
                    memmove(buff + i + 2, buff + i + 1, size-i-1);
                    buff[i+1] = 0;
                    ++size;
                }
                break;
            default:
                assert(false);
        }
    }
}
void Ais::constructHDLCFrame(uint8_t *buff, uint16_t &size)
{
    /*
     * As a class B "CS" transponder, we don't transmit a full ramp byte because
     * we have to listen for a few bits into each slot for Clear Channel Assessment.
     * Also, this implementation only adds 3 bits for ramp down, just in case
     * our TX bit clock is a little lazy. Not what the ITU standard says, but no
     * reasonable receiver should care about ramp-down bits. It's only what goes
     * between the 0x7E markers that counts.
     */

    // Make room for 35 bits at the front
    memmove(buff+35, buff, size);
    size += 35;
    putBits(buff, 0xFF, 3);                             // 3 ramp bits. That's all we can afford.
    putBits(buff+3, 0b010101010101010101010101, 24);    // 24 training bits
    putBits(buff+27, 0x7e, 8);                          // HDLC start flag

    // Now append the end marker and ramp-down bits
    addBits(buff, size, 0x7e, 8);                       // HDLC stop flag
    addBits(buff, size, 0x00, 3);                       // Ramp down
}

void Ais::nrziEncode(uint8_t *buff, uint16_t &size)
{
    bool NRZI = 1;        // Arbitrarily starting with 1
    int mSize     = 0;
    uint16_t index ;
    uint8_t offset;

    for ( uint16_t i = 0; i < size; ++i ) {
        if ( buff[i] == 0 )NRZI ^= 1;
        index = mSize / 8;
        offset = mSize % 8;
        
        if ( NRZI) TXPacket[index] |= ( 1 << offset );
        ++mSize;
    }
    uint16_t rem = 8 - mSize % 8;
    for ( uint16_t i = 0; i < rem; ++i ){
      TXPacket[index] |= ( 0 << offset );
      ++mSize;
    }
    // The TXPacket is now populated with the sequence of bits that need to be sent
}
uint32_t Ais::coordinateToUINT32(double value)
{
    uint32_t val = fabs(value) * 600000;
    if ( value < 0.0 )
        val = ~val + 1;

    return val;
}
void Ais::AISMessage18encode()
{
    uint8_t mType = 18;
    uint8_t mRI= 0;
    latitude=((lat[0]-48)*6000000)+((lat[1]-48)*600000)+((lat[2]-48)*10000)+((lat[3]-48)*1000)+((lat[5]-48)*100)+((lat[6]-48)*10)+(lat[7]-48);
    if (north)latitude=-latitude;
    longitude=((lon[0]-48)*60000000)+((lon[1]-48)*6000000)+((lon[2]-48)*600000)+((lon[3]-48)*10000)+((lon[4]-48)*1000)+((lon[6]-48)*100)+((lon[7]-48)*10)+(lon[8]-48);
    if (east)longitude=-longitude;    //AISMessage::encode(station, packet);
    // TODO: Perhaps this shouldn't live on the stack?
    uint8_t payload[MAX_AIS_TX_PACKET_SIZE];
    uint16_t size = 0;
    uint32_t value;
    value = mType;
    addBits(payload, size, value, 6);   // Message type
    value = mRI;
    addBits(payload, size, value, 2);   // Repeat Indicator
    //value = MMSI;
    addBits(payload, size, MMSI, 30);  // MMSI
    value = 0;
    addBits(payload, size, value, 8);   // Spare bits
    value = (uint32_t)(sog * 10);
    addBits(payload, size, value, 10);  // Speed (knots x 10)
    value = 1;
    addBits(payload, size, value, 1);   // Position accuracy is high
    //value = coordinateToUINT32(Gpslookup.longitude);
    //value = (uint32_t)longitude;
    addBits(payload, size, longitude, 28);  // Longitude
    //value = coordinateToUINT32(Gpslookup.latitude);
    //value = (uint32_t)latitude;
    addBits(payload, size, latitude, 27);  // Latitude
    //value = (uint32_t)cog;
    addBits(payload, size, cog, 12);  // COG
    //value = 511;
    //value = (uint32_t)sog;
    //Serial.print("sog ");
    //Serial.println(sog);
    value = (uint32_t)sog/10;
    addBits(payload, size, value, 9);   // We don't know our heading
    //value = utc % 60;
    value = utc;
    addBits(payload, size, utc, 6);   // UTC second. Possibly incorrect.
    value = 0;
    addBits(payload, size, value, 2);   // Spare
    value = 1;
    addBits(payload, size, value, 1);   // We are a "CS" unit
    value = 0;
    addBits(payload, size, value, 1);   // We have no display
    value = 0;
    addBits(payload, size, value, 1);   // We have no DSC
    value = 0;
    addBits(payload, size, value, 1);   // We don't switch frequencies so this doesn't matter
    value = 0;
    addBits(payload, size, value, 1);   // We do not respond to message 22 to switch frequency
    value = 0;
    addBits(payload, size, value, 1);   // We operate in autonomous and continuous mode
    value = 0;
    addBits(payload, size, value, 1);   // No RAIM
    value = 1;
    addBits(payload, size, value, 1);   // We use ITDMA (as a CS unit)
    value = DEFAULT_COMM_STATE;
    addBits(payload, size, value, 19);  // Standard communication state flag for CS units
    //ASSERT(size == 168);

    finalize(payload, size);
    for ( uint16_t i = 0; i < size; ++i ) {
      //Serial.print(payload[i],HEX);
      //Serial.print(",");
      payload[i]=0;
    }
    //Serial.println();
    size=0;
}
