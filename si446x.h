#ifndef si446x_h
#define si446x_h

#include <Arduino.h>
#include "radio_config.h"
class si446x
{
  public:
    si446x();
    void radioShutdown();
    void loadpatch();
    void configure();
    void Si4464_Init();
    void setFrequency( uint32_t freq, uint16_t shift);
    void radioTune(uint32_t frequency, uint16_t shift, int8_t level, uint16_t size); 
    void setPowerLevel(int8_t level);
    void startupradio();
    void Si4464_write(const byte pData[],int byteCountTx);
    void startTx(uint16_t size);
    void activateTX(uint8_t channel);
    void activateRX(uint8_t channel);
    void si4464interupt();
    void setShift( uint16_t shift);
   private:  
    void waitforreply();
    void waitforreplyread();
    
    uint8_t dBm2powerLvl(int32_t dBm);
    unsigned char interrupt[9];
};     
#endif
