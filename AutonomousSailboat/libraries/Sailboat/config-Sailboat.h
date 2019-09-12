#ifndef SAILBOAT_ALL_CONFIG_H
#define SAILBOAT_ALL_CONFIG_H
//Choose the boat here
#define SAILBOAT_PRO
#define VERSION_ARDUINO "2.0"

#ifdef SAILBOAT_PRO
#pragma message("SAILBOAT_PRO chosen")
#include <config-SailboatPRO.h>
#endif
#ifdef SAILBOAT
#pragma message("SAILBOAT chosen")
#include <config-SailboatSmall.h>
#endif
#ifdef SAILBOAT_BIG
#pragma message("SAILBOAT_BIG chosen")
#include <config-SailboatBig.h>
#endif
#ifdef SAILBOAT_CATAMARAN
#pragma message("SAILBOAT_CATAMARAN chosen")
#include <config-Catamaran.h>
#endif
#ifdef SAILBOAT_TRIMARAN
#pragma message("SAILBOAT_TRIMARAN chosen")
#include <config-Trimaran.h>
#endif

#include <config-RC.h>

/**CONTROLLERS**/
/***********/
#define NB_CONTROLLERS 7

#define STANDBY_CONTROLLER 0
#define RUDDERSAIL_CONTROLLER 1
#define RETURNHOME_CONTROLLER 2
#define HEADER_CONTROLLER 3
#define RC_CONTROLLER 4
#define SAILCAP_CONTROLLER 5
#define RUDDER_CONTROLLER 6


/*******COMMON CONFIG********/
#define EARTH_RADIUS	6371000  // Earth radius in metres

/**PIN**/
/********/
#ifdef SERVO_SHIELD
	#define SERVO_ADDRESS 0x40
	#define RUDDER_SERVO 0
	#define WINCH_SERVO 1
	#define RUDDER2_SERVO 2
#else
	//RUDDER
	#define RUDDER_PIN 6
	//SAIL
	#define WINCH_PIN 7
#endif
//WIND DIRECTION SENSOR
#define WIND_SENSOR_PIN A0
//BATTERY SENSOR
#define BATTERY_SENSOR_PIN A3

//GPS
#define GPS_SERIAL 1
#define GPS_BAUD_RATE	9600


/*MISC*/
/******/
//PIN//
//////
//LCD I2C
#define LOGGER 0
#define LCD_SCREEN_ADDRESS    0x3f
// RTC Module: I2C bus
// SDA pin 20
// SCL pin 21


// SD card: SD card attached to SPI bus as follows:
#ifdef SD_ACTIVATED
  // SD module:   |    Arduino:
  //    MISO    =====    PIN 50 / ICSP 1
  //    MOSI    =====    PIN 51 / ICSP 4
  //    CLK     =====    PIN 52 / ICSP 3
#define SD_PIN_CHIP_SELECT 53  // CS
#endif

#endif
