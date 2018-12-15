#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "si446x.h"
#include "si446x_patch.h"
#include "radio_config.h"
#define OSC_FREQ        30000000
uint32_t outdiv;
//#define IRQPin 2
//#define CLSpin 5 //gpio1
//#define SDNPin 8
//#define chipSelectPin  4
//#define DEBUG 1
//9 gpoi 0 data
//#define SI446x_patch_cmds{ SI446X_PATCH_CMDS}


volatile boolean interuptwaiting;
volatile boolean transmit;
volatile boolean transmitbit;
volatile boolean timerwaiting;
unsigned char TCCR0A_save;
unsigned char TCCR0B_save;
unsigned char TCCR2A_save;
unsigned char TCCR2B_save;
volatile int timer;

const unsigned char Si446xPatchCommands[][8] PROGMEM ={SI446X_PATCH_CMDS};
si446x::si446x(){}
void si446x::loadpatch()
{
  unsigned char configvalue;  
  delay(100);
  digitalWrite(SDNPin, HIGH);
  delay(100);
  digitalWrite(SDNPin, LOW);
  //while (!(digitalRead(CLSpin)));
  delay(1000);
 #ifdef DEBUG
    Serial.print((sizeof(Si446xPatchCommands) / sizeof(Si446xPatchCommands[0])),DEC);
    Serial.println(" Loading SI446X_PATCH_CMDS");
 #endif
  for (int i = 0; i < (sizeof(Si446xPatchCommands) / sizeof(Si446xPatchCommands[0])); i++){
    delay(2);
    digitalWrite(chipSelectPin, LOW);
    for (int j = 0; j < sizeof(Si446xPatchCommands[i]) ; j++){
     configvalue= pgm_read_byte(&Si446xPatchCommands[i][j]);
     SPI.transfer(configvalue);
     #ifdef DEBUG
     Serial.print(j,DEC); 
     Serial.print(" ");
     Serial.print(configvalue,HEX);
     Serial.print(" ");
     #endif
    }
  digitalWrite(chipSelectPin, HIGH);
  #ifdef DEBUG      
  Serial.println();
  #endif
  }
  
}
void si446x::configure()
{
  delay(100);
  digitalWrite(chipSelectPin, HIGH);
  delay(100);
  digitalWrite(SDNPin, HIGH);
  delay(100);
  digitalWrite(SDNPin, LOW);
  while (!(digitalRead(CLSpin)));

 #ifdef DEBUG
    Serial.println("Starting RADIO_CONFIGURATION");
    delay(200);
 #endif 
  

  unsigned char buff[17];
  unsigned char buffpos=0;
  unsigned char configvalue;
  unsigned char configlength;
  unsigned int configpos=0;
  unsigned int configendpos=0;
  unsigned int configcurrentpos=0;
  static const uint8_t config[] PROGMEM =RADIO_CONFIGURATION_DATA_ARRAY;

 #ifdef DEBUG
    Serial.println("Loading RADIO_CONFIGURATION_DATA_ARRAY");
    delay(200);
 #endif
  
  while(configpos < sizeof(config)){
    configlength=pgm_read_byte(&config[configpos]);
    configpos++;
    configcurrentpos=configpos;
    configendpos=configpos+configlength;
    buffpos=0;
    #ifdef DEBUG
    Serial.println(configlength,DEC);
    delay(200);
    #endif
    for(unsigned int i=configcurrentpos;i<configendpos;i++)// load each value from config
    { 
      configvalue= pgm_read_byte(&config[i]);
      buff[buffpos++]=configvalue;
      #ifdef DEBUG
        Serial.print(configvalue,HEX);
        Serial.print(" ");
      #endif
      configpos++;
    }
    Si4464_write(buff,configlength);
    #ifdef DEBUG      
     Serial.println();
    #endif 
  }
  #ifdef DEBUG      
     Serial.println("coeff update");
  #endif
    // Set AIS filter
      /*
     * Set the TX filter coefficients for a BT of 0.4 as per ITU spec for AIS
     * http://community.silabs.com/t5/Wireless/si4463-GMSK-spectrum-spread/m-p/160063/highlight/false#M9438
     */
  uint8_t coeff[] = {0x11, 0x20, 0x09, 0x0F, 0x52, 0x4f, 0x45, 0x37, 0x28, 0x1a, 0x10, 0x09, 0x04};
  Si4464_write(coeff, 13); 

}

void si446x::startupradio() {
  pinMode(IRQPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  //pinMode(TXon, OUTPUT);
  //pinMode(RXon, OUTPUT);
  pinMode(CLSpin, INPUT);
  pinMode(SDNPin, OUTPUT);
  
  // TIMER 1 for interrupt frequency 9603.841536614646 Hz:
  /*cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 9603.841536614646 Hz increments
  OCR1A = 1665; // = 16000000 / (1 * 9603.841536614646) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts

  TCCR0A_save=TCCR0A;
  TCCR0B_save=TCCR0B;
  TCCR2A_save=TCCR2A;
  TCCR2B_save=TCCR2B;
    */
  digitalWrite(SDNPin, LOW);
  digitalWrite(chipSelectPin, HIGH);
  delay(1000);
  #ifdef DEBUG
     Serial.println("Setup radio");
 #endif 
}

/*
void si446x::delay_time(int Time_ms) {
  timer=0;
  timerwaiting=1;
  int Time=Time_ms*3.913;
  while(timer < Time){asm("nop");}
  timerwaiting=0;
  digitalWrite(10,digitalRead(10) ^ 1);
}
*/
void si446x::Si4464_write(const byte pData[],int byteCountTx) {
byte wordb;
  digitalWrite(chipSelectPin, LOW);
    for (int j = 0; j < byteCountTx; j++) // Loop through the bytes of the pData
    {
      wordb = pData[j];
      SPI.transfer(wordb);
    }
  digitalWrite(chipSelectPin, HIGH);
  while (!(digitalRead(CLSpin)));
  //waitforreply();
}

void si446x::waitforreply(){
  unsigned char reply;
    reply=0;
    int counter =0;
    while (reply != 0xFF)
    {
      digitalWrite(chipSelectPin, LOW);
      //delay(1);
      SPI.transfer(0x44);
      //delay(1);
      reply = SPI.transfer(0x00);
      //delay(1);
      digitalWrite(chipSelectPin, HIGH);
      delay(10);
      counter++;
      if (counter == 2000)reply = 0xFF;
    }
    delay(1);
}

void si446x::waitforreplyread(){
  unsigned char reply;
  int counter =0;
    reply=0;
    while (reply != 0xFF)
    {
      counter++;
      if (counter == 2000)
      digitalWrite(chipSelectPin, LOW);
      delay(1);
      SPI.transfer(0x44);
      delay(1);
      reply = SPI.transfer(0x00);
      delay(1);
      if (reply != 0xFF){
        digitalWrite(chipSelectPin, HIGH);
        delay(1);
      }
      counter++;
      if (counter == 2000)reply = 0xFF;
    }
    delay(1);
    digitalWrite(chipSelectPin, HIGH);
}
void si446x::si4464interupt() {
  unsigned char reply;
  reply=0;
  //void waitforreply();
  Si4464_write((const byte[]){0x20,0x00,0x00,0x00 },4); //reply 0x04,0x04,0x00,0x02,0x00,0x00,0x14,0x14,0x00
  delay(1);
  while (reply != 0xFF)
    {
      digitalWrite(chipSelectPin, LOW);
      delay(1);
      SPI.transfer(0x44);
      delay(1);
      reply = SPI.transfer(0x00);
      delay(1);
      if (reply != 0xFF){
        digitalWrite(chipSelectPin, HIGH);
        delay(1);
      }
    }
    delay(1);
  for (int j = 0; j < 9; j++) // Loop through the bytes of the pData
     {
         si446x::interrupt[j]=SPI.transfer(0x00);
     }
    delay(1);
   digitalWrite(chipSelectPin, HIGH);
   interuptwaiting =1;
}
void si446x::radioShutdown() {
  digitalWrite(SDNPin, HIGH);
  //digitalWrite(TXon, LOW);
  //digitalWrite(RXon, HIGH);
  //RADIO_SDN_SET(radio, true); // Power down chip
  //RF_GPIO1_SET(radio, false); // Set GPIO1 low
  //initialized[radio] = false;
}
void si446x::Si4464_Init(){
SPI.begin();
digitalWrite(SDNPin, HIGH);
digitalWrite(SDNPin, LOW);
while (!(digitalRead(CLSpin)));
delay(100);

//Serial.println("Init Start");
  // Power up (transmits oscillator type)
  uint8_t x3 = (OSC_FREQ >> 24) & 0x0FF;
  uint8_t x2 = (OSC_FREQ >> 16) & 0x0FF;
  uint8_t x1 = (OSC_FREQ >>  8) & 0x0FF;
  uint8_t x0 = (OSC_FREQ >>  0) & 0x0FF;
  uint8_t init_command[] = {0x02, 0x01, 0x01, x3, x2, x1, x0};
  Si4464_write(init_command, 7);
  //waitforreply();
  // Set transmitter GPIOs
/*  uint8_t gpio_pin_cfg_command[] = {
    0x13, // Command type = GPIO settings
    0x44, // GPIO0        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
    0x00, // GPIO1        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
    0x00, // GPIO2        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
    0x00, // GPIO3        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
    0x00, // NIRQ
    0x00, // SDO
    0x00  // GEN_CONFIG
};
   */
  uint8_t gpio_pin_cfg_command[] = {0x13, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  //uint8_t gpio_pin_cfg_command[] = {0x13, 0x1A, 0x00, 0x11, 0x14, 0x1B, 0x00, 0x00};
  Si4464_write(gpio_pin_cfg_command, 8);
    //waitforreply();
    // Disable preamble
  uint8_t disable_preamble[] = {0x11, 0x10, 0x01, 0x00, 0x00};
  Si4464_write(disable_preamble, 5);
  
  //waitforreply();

  // Do not transmit sync word
  uint8_t no_sync_word[] = {0x11, 0x11, 0x01, 0x00, (0x01 << 7)};
  Si4464_write(no_sync_word, 5);
  
  //waitforreply();
  // Setup the NCO modulo and oversampling mode
  uint32_t s = OSC_FREQ / 10;
  uint8_t f3 = (s >> 24) & 0xFF;
  uint8_t f2 = (s >> 16) & 0xFF;
  uint8_t f1 = (s >>  8) & 0xFF;
  uint8_t f0 = (s >>  0) & 0xFF;
  uint8_t setup_oversampling[] = {0x11, 0x20, 0x04, 0x06, f3, f2, f1, f0};
  Si4464_write(setup_oversampling, 8);
  //waitforreply();
  // Setup the NCO data rate for Transmission
  uint8_t setup_data_rate[] = {0x11, 0x20, 0x03, 0x03, 0x00, 0x11, 0x30};
  Si4464_write(setup_data_rate, 7);
  //waitforreply();

  // Use 2GFSK from async GPIO0
  uint8_t use_2gfsk[] = {0x11, 0x20, 0x01, 0x00, 0x0B};
  Si4464_write(use_2gfsk, 5);
  //waitforreply();
  // Set AIS filter
      /*
     * Set the TX filter coefficients for a BT of 0.4 as per ITU spec for AIS
     * http://community.silabs.com/t5/Wireless/si4463-GMSK-spectrum-spread/m-p/160063/highlight/false#M9438
     */
  uint8_t coeff[] = {0x11, 0x20, 0x09, 0x0F, 0x52, 0x4f, 0x45, 0x37, 0x28, 0x1a, 0x10, 0x09, 0x04};
  Si4464_write(coeff, 13);
  //waitforreply();
  // Set AFSK filter
  /*
  uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};

  uint8_t i;
  for(i=0; i<sizeof(coeff); i++) {
    uint8_t msg[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
    Si4464_write(msg, 5);
    waitforreply();
  }
*/
  //Serial.println("Init Done");
}

void si446x::setFrequency( uint32_t freq, uint16_t shift) {
  // Set the output divider according to recommended ranges given in Si4464 datasheet
  uint32_t band = 0;
  if(freq < 705000000UL) {outdiv = 6;  band = 1;};
  if(freq < 525000000UL) {outdiv = 8;  band = 2;};
  if(freq < 353000000UL) {outdiv = 12; band = 3;};
  if(freq < 239000000UL) {outdiv = 16; band = 4;};
  if(freq < 177000000UL) {outdiv = 24; band = 5;};

  // Set the band parameter
  uint32_t sy_sel = 8;
  uint8_t set_band_property_command[] = {0x11, 0x20, 0x01, 0x51, (band + sy_sel)};
  Si4464_write(set_band_property_command, 5);
  //waitforreply();
  // Set the PLL parameters
  uint32_t f_pfd = 2 * OSC_FREQ / outdiv;
  uint32_t n = ((uint32_t)(freq / f_pfd)) - 1;
  float ratio = (float)freq / (float)f_pfd;
  float rest  = ratio - (float)n;

  uint32_t m = (uint32_t)(rest * 524288UL);
  uint32_t m2 = m >> 16;
  uint32_t m1 = (m - m2 * 0x10000) >> 8;
  uint32_t m0 = (m - m2 * 0x10000 - (m1 << 8));

  uint32_t channel_increment = 524288 * outdiv * shift / (2 * OSC_FREQ);
  uint8_t c1 = channel_increment / 0x100;
  uint8_t c0 = channel_increment - (0x100 * c1);

  uint8_t set_frequency_property_command[] = {0x11, 0x40, 0x04, 0x00, n, m2, m1, m0, c1, c0};
  Si4464_write(set_frequency_property_command, 10);
  //waitforreply();
  uint32_t x = ((((uint32_t)1 << 19) * outdiv * 1300.0)/(2*OSC_FREQ))*2;
  uint8_t x2 = (x >> 16) & 0xFF;
  uint8_t x1 = (x >>  8) & 0xFF;
  uint8_t x0 = (x >>  0) & 0xFF;
  uint8_t set_deviation[] = {0x11, 0x20, 0x03, 0x0a, x2, x1, x0};
  Si4464_write(set_deviation, 7);
  //waitforreply();
}

void si446x::setShift( uint16_t shift) {
  if(!shift)
    return;

  float units_per_hz = (( 0x40000 * outdiv ) / (float)OSC_FREQ);

  // Set deviation for 2FSK
  uint32_t modem_freq_dev = (uint32_t)(units_per_hz * shift / 2.0 );
  uint8_t modem_freq_dev_0 = 0xFF & modem_freq_dev;
  uint8_t modem_freq_dev_1 = 0xFF & (modem_freq_dev >> 8);
  uint8_t modem_freq_dev_2 = 0xFF & (modem_freq_dev >> 16);

  uint8_t set_modem_freq_dev_command[] = {0x11, 0x20, 0x03, 0x0A, modem_freq_dev_2, modem_freq_dev_1, modem_freq_dev_0};
  Si4464_write(set_modem_freq_dev_command, 7);
  waitforreply();
}
uint8_t si446x::dBm2powerLvl(int32_t dBm) {
  if(dBm < -35) {
    return 0;
  } else if(dBm < -7) {
    return (uint8_t)((2*dBm+74)/15);
  } else if(dBm < 2) {
    return (uint8_t)((2*dBm+26)/3);
  } else if(dBm < 8) {
    return (uint8_t)((5*dBm+20)/3);
  } else if(dBm < 13) {
    return (uint8_t)(3*dBm-4);
  } else if(dBm < 18) {
    return (uint8_t)((92*dBm-1021)/5);
  } else {
    return 127;
  }
}
void si446x::setPowerLevel(int8_t level) {
  // Set the Power
  uint8_t set_pa_pwr_lvl_property_command[] = {0x11, 0x22, 0x01, 0x01, dBm2powerLvl(level)};
  Si4464_write(set_pa_pwr_lvl_property_command, 5);
  waitforreply();
}

void si446x::startTx(uint16_t size) {
  digitalWrite(SDNPin, LOW);
  digitalWrite(chipSelectPin, HIGH);
  delay(1000);
  uint8_t change_state_command[] = {0x31, 0x00, 0x30, (size >> 8) & 0x1F, size & 0xFF};
  Si4464_write(change_state_command, 5);
  
}
void si446x::activateRX(uint8_t channel){
    byte GPIO_SetBits[]={0x13, 0x1A, 0x00, 0x11, 0x14, 0x1B, 0x00, 0x00};
    Si4464_write(GPIO_SetBits,8);
    uint8_t buf[] = { 0x20, 0,0,0 };    
    Si4464_write(buf,4);
    byte packet[] = { 0x32, channel,0,0,0,0,0 };
    Si4464_write(packet,7);
  
}
void si446x::activateTX(uint8_t channel)
{   
 //byte GPIO_SetBits[]={0x13,0x04,0x00,0x10,0x13,0x1A,0x00,0x00};
 //byte GPIO_SetBits[]={0x13,0x00,0x00,0x10,0x04,0x1A,0x00,0x00};
   byte GPIO_SetBits[]={0x13,0x00,0x00,0x10,0x04,0x00,0x00,0x00};

    Si4464_write(GPIO_SetBits,8);
    uint8_t buf[] = { 0x20, 0,0,0 };    
    Si4464_write(buf,4);
    byte packet[] = { 0x31, channel,0, 0,0,0 };
    Si4464_write(packet,6);
}
void si446x::radioTune(uint32_t frequency, uint16_t shift, int8_t level, uint16_t size) {
  setFrequency(frequency, shift);  // Set frequency
  setShift(shift);         // Set shift
  setPowerLevel(level);      // Set power level
  //digitalWrite(TXon, HIGH);
  digitalWrite(SDNPin, LOW);
  startTx(size);
}
