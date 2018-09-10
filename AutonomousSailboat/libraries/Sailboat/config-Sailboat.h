#define SAILBOAT
#define VERSION_ARDUINO "1.2"

#ifdef SAILBOAT
#define HK_TR6
#include <config-SailboatSmall.h>
#endif
#ifdef SAILBOAT_BIG
#define FLYSKY
#include <config-SailboatBig.h>
#endif
#ifdef SAILBOAT_CATAMARAN
#define FLYSKY
#include <config-Catamaran.h>
#endif
#ifdef SAILBOAT_TRIMARAN
#define HK_TR6
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
//RUDDER
#define RUDDER_PIN 6
//SAIL
#define WINCH_PIN 7
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
