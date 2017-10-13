#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define NB_SENSORS		2

#define SENSOR_WINDSENSOR	0
#define SENSOR_GPS			1

// ####################################################
// #                                                  #
// #                       PINS                       #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                    ANALOG PINS                   =
//  =                                                  =
//  ====================================================
// Wind sensor:
#define WIND_SENSOR_PIN A0


//  ====================================================
//  =                                                  =
//  =                    DIGITAL PINS                  =
//  =                                                  =
//  ====================================================
// Rudder servo:
#define RUDDER_PIN 7   // Pin 7 Connected to the servo moving the rudder


// Winch/Sail servo:
#define WINCH_PIN 8  // Pin 8 Connected to the servo moving the sail through the winch


// RC receiver:
#define RC_PIN_1 10   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_PIN_2 11   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_PIN_3 12   // Pin 12 Connected to Channel-5 of Transmitter


// IMU:
// Linked to the I2C bus
// The IMU requires a I2C bus at 3.3V, so you need to use a level-shifter ONLY for the I2C bus (SDA/SCL)
// It's power supply must be 5V.
// Wiring:
//  IMU:                |              Arduino:
//  VCC  ==============================  5V
//  GND  ==============================  GND
//              Level-shifter: 
//  SDA  =======  LV1 | HV1  ==========  SDA   
//  SCL  =======  LV2 | HV2  ==========  SCL
//                      HV   ==========  5V
//                      LV   ==========  3.3V
//                      GND  ==========  GND
//                      GND  ==========  GND  !! WARNING !! 
//                                            Both ground of the level-shifter have to be linked to the Arduino GND 


// GPS: Linked to the serial port 1
// The GPS requires a serial port to run well, so it can't be declared with one pin on 
// the Rx and the other on the Tx. We need to use the Serial1 variable for it.
// Be careful, it requires 3.3V for it's power supply NOT 5V !
// It seems to be advised also to use the serial port with 3.3V! -> level-shifter (see IMU wiring above)
// (see http://www.ayomaonline.com/iot/gy-gps6mv2-neo6mv2-neo-6m-gps-module-with-arduino-usb-ttl/)
//
// Wiring:
//  GPS:   |    Arduino:
//   RX  =====   TX1 (18)  // Useless because we don't send data to the GPS
//   TX  =====   RX1 (19)
//  VCC  =====   3.3V
//  GND  =====   GND
#define GPS_RX 18
#define GPS_TX 19
#define GPS_BAUD_RATE	9600
#define EARTH_RADIUS	6371000  // Earth radius in metres

// RTC Module: I2C bus
// SDA pin 20
// SCL pin 21


// LCD screen: I2C bus (same as RTC Module)
// SDA pin 20
// SCL pin 21


// Ultrasonic sensor: HC-SR04  // TODO: Must be changed because it is not waterproof
/*// Left sensor:
#define ULTRA_SONIC_TRIGG_PIN_L 30  //appoint trigger pin for ultrasonic Left
#define ULTRA_SONIC_ECHO_PIN_L 31  //appoint echo pin for ultrasonic Left

// Centre sensor:
#define ULTRA_SONIC_TRIGG_PIN_C 32  //appoint trigger pin for ultrasonic Centre
#define ULTRA_SONIC_ECHO_PIN_C 33    //appoint echo pin for ultrasonic Centre

// Right sensor:
#define ULTRA_SONIC_TRIGG_PIN_R 34  //appoint trigger pin for ultrasonic Right
#define ULTRA_SONIC_ECHO_PIN_R 35    //appoint echo pin for ultrasonic Right */


// Reset pin:
// This pin needs to be linked to the reset one:
//  Arduino:
//   42  ======  reset
//
// !!!!!!!! BE CAREFUL !!!!!!!!:
// You need to plug this pin after having switched ON the Arduino board. 
// Then you need to unplug it before uploading a new program.
#define RESET_PIN 42  // The answer to life, the universe and everything... The RESET button! =D


// SD card: SD card attached to SPI bus as follows:
#ifdef SD_ACTIVATED
  // SD module:   |    Arduino:
  //    MISO    =====    PIN 50 / ICSP 1
  //    MOSI    =====    PIN 51 / ICSP 4
  //    CLK     =====    PIN 52 / ICSP 3
#define SD_PIN_CHIP_SELECT 53  // CS
#endif


// Real Time Clock: DS3231 module attached to I2C bus as follows:
// SD module:   |    Arduino:
//    SDA    =====    SDA
//    SCL    =====    SCL
//    GND    =====    GND
//    VCC    =====    3.3V !!!!!!!!!!! Be careful, it is supposed to be 
//                    5V, but it does not work with 5V, due to overpowering


#endif