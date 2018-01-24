#ifndef CONFIG_H
#define CONFIG_H

/**CONTROLLERS**/
/***********/
#define NB_CONTROLLERS 6

#define STANDBY_CONTROLLER 0
#define RUDDERSAIL_CONTROLLER 1
#define RETURNHOME_CONTROLLER 2
#define HEADER_CONTROLLER 3
#define RC_CONTROLLER 4
#define C_CONTROLLER 5

/**SENSORS**/
/***********/
#define NB_SENSORS		3

#define SENSOR_WINDSENSOR	0
#define SENSOR_GPS			1
#define SENSOR_IMU			2

#define NB_SENSORS_NOT_ROS		1
#define SENSOR_RC			0
//PIN//
//////
//WIND
#define WIND_SENSOR_PIN A0
//GPS
#define GPS_RX 10
#define GPS_TX 11

//CONFIG//
/////////
//GPS
#define GPS_BAUD_RATE	9600
#define EARTH_RADIUS	6371000  // Earth radius in metres


/**ACTUATORS**/
/***********/
#define NB_ACTUATORS		2

#define ACTUATOR_RUDDER		0
#define ACTUATOR_SAIL		1

//PIN//
//////
//RUDDER
#define RUDDER_PIN 7
//SAIL
#define WINCH_PIN 8

/*MISC*/
/******/
#define RC_1 0   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_3 2   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_5 4   // Pin 12 Connected to Channel-5 of Transmitter
//PIN//
//////
//LCD I2C
#define LCD_SCREEN_ADDRESS	0x3f
//RC
#define RC_PIN_1 10   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_PIN_3 11   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_PIN_5 12   // Pin 12 Connected to Channel-5 of Transmitter
#define RC_NUM_CHANNELS 6
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