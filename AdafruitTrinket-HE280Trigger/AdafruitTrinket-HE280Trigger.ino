#include <TinyWireM.h>

//--------------------------------------------------------------------
// Sparkfun Trinket 3.3v to HE280 interface
//
// MHackney 10/17/2016
// Uses some Open Source code developed by SeeMeCNC in their Repetier fork to support the HE280
// Uses the TinyWireM I2Ccommunication library
//
// Function: process the HE280's accelerometer (IIS2DH) signal and report it 
// to the Duet controller as a binary on/off signal mimicing an endstop switch
//
// YAAA - yet another accelerometer adapter
//
// Sparkfun Trinket 3.3V spec sheet: https://learn.adafruit.com/introducing-trinket/pinouts
// IIS2DH spec sheet: http://www.st.com/en/mems-and-sensors/iis2dh.html
//
// Windows users must install drivers (https://learn.adafruit.com/introducing-trinket/windows-setup)
//
// Trinket setup and programming information: https://learn.adafruit.com/introducing-trinket/starting-the-bootloader
//
// Arduino IDE Setup
// ----------------------------
// Board: Adafruit Trinket 8MHz
// Programmer: USBtinyISP
//
// Although not tested, this should work with Smoothie too
//
//--------------------------------------------------------------------
//
//  Duet                 Trinket Pins             HE280 Pins
//
//                                                  1 ---> Fan
//                                                  2 ---> 12V 
//                                                  3 ---> Hot End
//                                                  4 ---> GND 
//                        #2 - SCL      <=======>   5                            BLACK
//                        #0 - SDA      <=======>   6                            RED
//                        #4 - INT      <--------   7                            BLUE
//                                                  8 ---> Thermistor
//
// Endstop Vcc <========> BAT
// Endstop GND <========> GND
// _Probe_In   <========> #1
//--------------------------------------------------------------------
//
// Trinket pins
//
// #0   - SDA - from HE280 SDA (red wire)
// #1   - OUTPUT_PIN - to controller endstop signal
// #2   - SCL- from HE280 SCL (black wire)
// #3   - LED
// #4   - INTERRUPT_PIN - from HE280 interrupt (blue wire)
// #RST - [NC]/NO select - NOTE: this is the reset pin and must be disabled as such
// BAT  - power from controller endstop Vcc
// GND  - ground from controller endstop ground
//--------------------------------------------------------------------

#define OUTPUT_PIN 1
#define LED_PIN 3
#define INTERRUPT_PIN 4

#define Z_PROBE_SENSITIVITY 25

#define IIS2DH_ADDRESS 0x19

// IIS2DH device registers
#define OUT_TEMP_L 0x0c
#define OUT_TEMP_H 0x0d

#define WHO_AM_I 0x0f
#define TEMP_CFG_REG 0x1f

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25

#define STATUS_REG 0x27

#define OUT_Z_H 0x2d // r

#define INT1_CFG 0x30 // rw
#define INT1_SRC 0x31 // r
#define INT1_THS 0x32 // rw
#define INT1_DURATION 0x33 //rw

#define INT2_CFG 0x34 // rw
#define INT2_SRC 0x35 // r
#define INT2_THS 0x36 // rw
#define INT2_DURATION 0x37 // rw

#define CLICK_CFG 0x38 // rw
#define CLICK_SRC 0x39 // r
#define CLICK_THS 0x3a // rw

//--------------------------------------------------------------------

#define FLASH_DURATION 100
#define FLASH_PAUSE 500

volatile byte state = LOW;
volatile bool got_interrupt = false;
int i2c_error = false;
int init_error = false;
int counter = 0;

//--------------------------------------------------------------------
//
void setup()
{
  // setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setup output pin
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  accelerometer_init();
}

//--------------------------------------------------------------------
// send data to accelerometer
void accelerometer_send(uint8_t val)
{
  TinyWireM.beginTransmission(IIS2DH_ADDRESS);
  TinyWireM.write(val);
  
  if (TinyWireM.endTransmission(false)) {
    // signal an error
    i2c_error = true;
  }
}

//--------------------------------------------------------------------
// write to a register
void accelerometer_write(uint8_t reg, uint8_t val)
{
  TinyWireM.beginTransmission(IIS2DH_ADDRESS);
  TinyWireM.write(reg);
  TinyWireM.write(val);
  if (TinyWireM.endTransmission()) {
    // signal an error
    i2c_error = true;
  }
}

//--------------------------------------------------------------------
// recieve without returning the data
void accelerometer_recv(uint8_t reg)
{
  uint8_t receiveByte;

  accelerometer_send(reg); // register to be read
  
  TinyWireM.requestFrom(IIS2DH_ADDRESS, 1); // request one 8 bit response
  
  if (TinyWireM.available()) {
    receiveByte = TinyWireM.read();  
  } else {
    // signal an error
    i2c_error = true;
  }
}

//--------------------------------------------------------------------
// read and return the data
uint8_t accelerometer_read(uint8_t reg)
{
  uint8_t receiveByte;

  accelerometer_send(reg); // register to be read
  
  TinyWireM.requestFrom(IIS2DH_ADDRESS, 1); // request one 8 bit response
  
  if (TinyWireM.available()) {
    receiveByte = TinyWireM.read();     
  } else {
    // signal an error
    i2c_error = true;
  }
  return receiveByte;
}

//--------------------------------------------------------------------
// Initialize the accelerometer
// SeeMeCNC source code:
void accelerometer_init()
{
  TinyWireM.begin(); // join I2C bus

  accelerometer_recv(WHO_AM_I); // shoud be 0x6A
   
  accelerometer_recv(INT1_SRC); // read to clear the interrupt source

  // setup CTRL_REG1
  accelerometer_recv(CTRL_REG1);
  accelerometer_write(CTRL_REG1, 0b10011100); // ODR 5.376kHz in LPMode [7-4]. Low power enable [3]. Z enable [2].
  accelerometer_recv(CTRL_REG1);

  // setup CTRL_REG3
  accelerometer_recv(CTRL_REG3);
  accelerometer_write(CTRL_REG3, 0b01000000); // CLICK interrupt on INT1 pin [7]. AOI (And Or Interrupt) on INT1 en [6]. AOI on INT2 en [5].
  accelerometer_recv(CTRL_REG3);

  // setup CTRL_REG6
  accelerometer_recv(CTRL_REG6);
  accelerometer_write(CTRL_REG6, 0b00000000); // CLICK interrupt on INT2 pin [7]. Interrupt 1 function enable on INT2 pin [6]. Interrupt 2 on INT2 pin enable [5]. 0=INT Active High [1]. 
  accelerometer_recv(CTRL_REG6);

  // setup CTRL_REG4
  accelerometer_recv(CTRL_REG4);
  accelerometer_write(CTRL_REG4, 0b00110000); // Full-scale selection 16G [5-4]. High resolution mode [3].
  accelerometer_recv(CTRL_REG4);

  // setup CTRL_REG5
  accelerometer_recv(CTRL_REG5);
  accelerometer_write(CTRL_REG5, 0b01001010); // FIFO enable [6]. Latch INT1 [3]. Latch INT2 until cleared by read [1].
  accelerometer_recv(CTRL_REG5);
  
  // setup INT1_CFG
  accelerometer_recv(INT1_CFG);
  accelerometer_write(INT1_CFG, 0b00100000); // ZHIE enabled [5]. ZLOE enabled [4].
  accelerometer_recv(INT1_CFG);

  // setup INT1_SRC - read the INT1 source, clears INT1_SRC IA (interrupt active) bit
  accelerometer_recv(INT1_SRC);
  
  // setup INT1_THS
  accelerometer_recv(INT1_THS);
  accelerometer_write(INT1_THS, Z_PROBE_SENSITIVITY);
  accelerometer_recv(INT1_THS);

  // setup INT1_DURATION
  accelerometer_recv(INT1_DURATION);
  accelerometer_write(INT1_DURATION, 0);
  accelerometer_recv(INT1_DURATION);

  // setup INT2_CFG
  accelerometer_recv(INT2_CFG);
  accelerometer_write(INT2_CFG, 0b00000000); // ZHIE not enabled on INT2 [5].
  accelerometer_recv(INT2_CFG);
  
  // setup INT2_THS
  accelerometer_recv(INT2_THS);
  accelerometer_write(INT2_THS, 50); // 7bits
  accelerometer_recv(INT2_THS);

  // setup INT2_DURATION
  accelerometer_recv(INT2_DURATION);
  accelerometer_write(INT2_DURATION, 0);
  accelerometer_recv(INT2_DURATION);
  
  // setup CLICK_CFG
  accelerometer_recv(CLICK_CFG);
  accelerometer_write(CLICK_CFG, 0b00010000); // single click Z axis
  accelerometer_recv(CLICK_CFG);
  
  // setup CLICK_SRC - read the CLICK interrupt source, clears the CLICK_SRC IA (interrupt active) bit
  accelerometer_recv(CLICK_SRC);
  
  // setup CLICK_THS
  accelerometer_recv(CLICK_THS);
  accelerometer_write(CLICK_THS, 50);
  accelerometer_recv(CLICK_THS);

  // Prepare for action
  
  // reenable CTRL_REG3 - AOI1 interrupt on INT1 pin (but nt CLICK ???)
  accelerometer_recv(CTRL_REG3);
  accelerometer_write(CTRL_REG3, 0b11000000); // CLICK interrupt on INT1 pin [7]. AOI (And Or Interrupt) on INT1 en [6]. AOI on INT2 en [5].
  accelerometer_recv(CTRL_REG3);

  // reset INT1_SRC - read the INT1 source, clears INT1_SRC IA (interrupt active) bit
  accelerometer_recv(INT1_SRC);

  // reenable CLICK_CFG - single click on Z
  accelerometer_recv(CLICK_CFG);
  accelerometer_write(CLICK_CFG, 0b00010000); // single click Z axis
  accelerometer_recv(CLICK_CFG);

  // reset CLICK_SRC - read the CLICK interrupt source, clears the CLICK_SRC IA (interrupt active) bit
  accelerometer_recv(CLICK_SRC);
}

//--------------------------------------------------------------------
// get status
void accelerometer_status()
{
    accelerometer_recv(0x31); // clears INT1_SRC IA (interrupt active) bit
    accelerometer_recv(0x35); // clears INT2_SRC IA (interrupt active) bit
    accelerometer_recv(0x39); // clears the CLICK_SRC IA (interrupt active) bit
    accelerometer_recv(0x2D); // read OUT_Z_H
}

//--------------------------------------------------------------------
//
void loop()
{
  if (init_error) {
    // accelerometer could not be initiallized, blink fast 3 times and pause forever
    digitalWrite(LED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
    delay(FLASH_PAUSE);
  } else {
    // accelerometer is initialized and running
    uint8_t int1_byte = accelerometer_read(INT1_SRC);
    if (int1_byte & 0b11000000)
    {
      digitalWrite(OUTPUT_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      delay(FLASH_DURATION);
      accelerometer_recv(INT1_SRC);
      digitalWrite(LED_PIN, LOW);
      digitalWrite(OUTPUT_PIN, LOW);
    }
  
    uint8_t click_byte = accelerometer_read(CLICK_SRC);
    if (click_byte & 0b11000000)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(FLASH_PAUSE);
      accelerometer_status();
      accelerometer_status();
      digitalWrite(LED_PIN, LOW);
    }
  }    
}




 
