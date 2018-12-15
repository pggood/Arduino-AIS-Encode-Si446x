/*
Using analogue pins for digital I/O is just the same as using digital ones.
A0 is referred to as Pin 14
A1 is referred to as Pin 15
A2 is referred to as Pin 16
A3 is referred to as Pin 17
A4 is referred to as Pin 18
A5 is referred to as Pin 19
*/
#define PPS_PIN 2
//switches
#define CHAT 14
#define CH_16 15
#define EMERGANCY 16
#define POWER_ON 17
//A1846S pins
#define DEBUG_PIN 6
#define DATA_PIN 7
#define dataclock_PIN 8
#define PWM_PIN 3
#define RESET_PIN 17
#define SWITCH_PIN 2
#define nSEN A1
#define CLK A5
#define DAT A4
//AH_MCP41xxx pins
#define CS   10   //chipselect pin
#define SHDN 9   //shutdown pin
#define RS   8   //reset pin
//si4463 pins
const int IRQPin =2;
const int GPIO0 = 9;
const int SDNPin = 8;
const int GPIO2 = 6;
//const int DATA_PIN = 7;
const int CLSpin=5; //gpio1
const int chipSelectPin = 4;
// Nokia Pins
const int scePin = 7;   // SCE - Chip select, pin 3 on LCD.
const int rstPin = 6;   // RST - Reset, pin 4 on LCD.
const int dcPin = 5;    // DC - Data/Command, pin 5 on LCD.
const int sdinPin = 11;  // DN(MOSI) - Serial data, pin 6 on LCD.
const int sclkPin = 13;  // SCLK - Serial clock, pin 7 on LCD.
const int blPin = 9;    // LED - Backlight LED, pin 8 on LCD.
