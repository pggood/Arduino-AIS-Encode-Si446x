#ifndef gpslookup_h
#define gpslookup_h

#include "Arduino.h"
class Gpslookup
{
  public:
    Gpslookup();
    bool gpsdata(char data);
    char lat[8];
    char lon[9];
    bool North;
    bool East;
    volatile uint32_t sog;
    volatile uint32_t cog;
    volatile uint32_t utc;
    volatile uint32_t GPSsec;
    volatile uint32_t GPSmin;
    volatile uint32_t GPShour;
  private:
    unsigned char charToHex(unsigned char c);
    char data;
    char i;
    char rxBuffer[10];
    char latBuffer[10];
    char lonBuffer[10];
    char rxWriteIndex;
    char rxValueIndex;
    char checksum;
    char xorcalc;
    char GPSday;
    char GPSmonth;
    char GPSyear;
    bool Status;
	  //char starting[6]={'$','G','P','G','G','A'};
    char starting[6]={'$','G','P','R','M','C'};

    bool dataReady;
    bool fix;
    bool checksumpos;
    bool bufferEnabled;
    bool xorcalcEnabled; 

	  bool LatitudeOffsetSign;
	  bool LongitudeOffsetSign;
};
#endif
