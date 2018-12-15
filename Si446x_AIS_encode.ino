/*
 * Library to control Silicon Laboratories SI6342 radio
 * Author: Peter Goodricke
 * based on dAISy project Adrian Studer ported to arduino
 * License: CC BY-NC-SA Creative Commons Attribution-NonCommercial-ShareAlike
 *       http://creativecommons.org/licenses/by-nc-sa/4.0/
 */
#include <SPI.h> 
#include "pins.h"
#include "ais.h"
#include "gpslookup.h"
#include "si446x.h"
#define SAMPLERATE 9600
//#define DATA_PIN 7
//#define DEBUG_PIN 6
#define DEBUG 1
//#define AFSK_ADC_INPUT 2
//97,26,91,58,10
//#define  MMSI {0x61,0x1A,0x5B,0x3A,0x0A}
//const unsigned char  MMSI[]={97,26,91,58,10};
const unsigned char  MMSI[]={9,72,69,15,81};
/*const int ClockPin =2;
const int IRQPin =3;
const int chipSelectPin = 4;
const int DEBUG_PIN = 5;
const int RXon = 5;
const int TXon = 5;
const int DATA_PIN = 9;

const int CLSpin=7;
const int SDNPin = 8;
*/
unsigned char dsc_header;
uint8_t checkoffset;
unsigned int chan_timer;
bool transmitdata=0;
bool GPSReadydata=0;
int bits=0;
char i;
Ais ais;
Gpslookup gpslookup;
si446x si446X;
//ISR(ADC_vect) {
void si4464interupt(){
  si446X.si4464interupt();
}

void si4464_9600(){ 
    bool NRZI;
    uint16_t index;
    uint8_t offset;
    static uint8_t tcnt = 0;

    //TIFR1 = _BV(ICF1); // Clear the timer flag

    if (transmitdata == 1) {
        digitalWrite(DEBUG_PIN,!digitalRead(DEBUG_PIN));
        //digitalWrite(DEBUG_PIN,1);
        //Serial.println("send ");
        //dataclock=!dataclock;
        //digitalWrite(dataclock_PIN, 1);
        //digitalWrite(dataclock_PIN, 0); 
        index = bits / 8;
        offset = bits % 8;
        NRZI = (ais.TXPacket[index] & (1 << offset)) != 0;
        digitalWrite(DATA_PIN, NRZI);
        
        bits++;
        if (bits > 255) {
            transmitdata = 0;
            bits = 0;
            //si446X.activateRX(19);          
            //si446X.radioShutdown();

          
        }
        digitalWrite(DEBUG_PIN,0);
    }
}

void setup() {
        // put your setup code here, to run once:
        SPI.begin();
        Serial.begin(9600);
        pinMode(DATA_PIN ,  OUTPUT); 
        pinMode(DEBUG_PIN ,  OUTPUT);    
        digitalWrite(DEBUG_PIN, HIGH);
        #ifdef DEBUG
        Serial.println("Setup Start");
        delay(1000);
        #endif 

        //si446X.Si4464_write((const byte[]){0x23,0x00},2);
        //si446X.Si4464_write((const byte[]){0x44,0x00,0x00,0x00,0x00},5);
      
      //attachInterrupt(digitalPinToInterrupt(IRQPin),si4464IRQ, RISING );

      //si446X.Si4464_write((const byte[]){0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00},8);
      chan_timer=0;
      attachInterrupt(digitalPinToInterrupt(2), si4464interupt, FALLING);
      attachInterrupt(digitalPinToInterrupt(3), si4464_9600, FALLING);
      // ADC as timer
/*      TCCR1A = 0;
      TCCR1B = _BV(CS11) | _BV(WGM13) | _BV(WGM12);
      ICR1 = ((F_CPU / 8) / 9600) - 1; //TODO: get the actual refclk from dds
      // NOTE: should divider be 1 or 8?
      ADMUX = _BV(REFS0) | _BV(ADLAR) | AFSK_ADC_INPUT; // Channel AFSK_ADC_INPUT, shift result left (ADCH used)
      ADCSRB = _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0);
      ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2); // | _BV(ADPS0);
*/      
      
      si446X.startupradio();
      si446X.configure();
      //si446X.Si4464_Init(); 
    //si446X.setFrequency(161975000,0);
      #ifdef DEBUG
        Serial.println("setFrequency");
        delay(1000);
      #endif 
      sei();
      si446X.setFrequency(161966165,0);
      uint8_t coeff[] = {0x11, 0x20, 0x09, 0x0F, 0x52, 0x4f, 0x45, 0x37, 0x28, 0x1a, 0x10, 0x09, 0x04};
      si446X.Si4464_write(coeff, 13);
      #ifdef DEBUG
      Serial.println("end");
      #endif
}

void loop() {
  char incomingByte;
  //Serial.println("Loop");
  char nmea[]="$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
  for (int j = 0; j < strlen(nmea); j++){
    if (gpslookup.gpsdata(nmea[j])){
  //if (Serial.available() > 0) {
  //  incomingByte = Serial.read();
   // if (gpslookup.gpsdata(incomingByte)){
      ais.MMSI        = MMSI[0]*100000000UL+MMSI[1]*1000000UL+MMSI[2]*10000UL+MMSI[3]*100UL+MMSI[4];      
      for (int i = 0; i < 8; i++)ais.lat[i] = gpslookup.lat[i];
      for (int i = 0; i < 9; i++)ais.lon[i] = gpslookup.lon[i];
      ais.north       = gpslookup.North;
      ais.east        = gpslookup.East;
      ais.sog         = gpslookup.sog;
      ais.cog         = gpslookup.cog;
      ais.utc         = gpslookup.GPSsec;
      ais.AISMessage18encode();
      #ifdef DEBUG
        for (int k = 0; k < 33; k++){
          Serial.print(ais.TXPacket[k],HEX);
          Serial.print(" ");
        }
        Serial.println();
        #endif
        GPSReadydata=1;
      }      
  }
  if (GPSReadydata==1){   
      Serial.println("activateTX");
      si446X.setFrequency(161966811,0);
      si446X.activateTX(0);      
      delay(300);
      transmitdata=1;
      delay(300);
      si446X.activateRX(0);
      delay(300);
      si446X.setFrequency(161966713,0);
      si446X.activateTX(1);      
      delay(300);
      transmitdata=1;
      delay(300);
      si446X.activateRX(1);
      delay(300);
      si446X.setFrequency(161966811,0);
      si446X.activateTX(0);      
      delay(300);
      transmitdata=1;
       delay(300);
      si446X.activateRX(0);
      delay(300);
      si446X.setFrequency(161966713,0);
      si446X.activateTX(1);      
      delay(300);
      transmitdata=1;
       delay(300);
      Serial.println("activateRX");
       si446X.activateRX(1);
      delay(4000);
      GPSReadydata=0;
  }
}

