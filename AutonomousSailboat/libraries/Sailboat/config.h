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
#define WIND_SENSOR_PIN A2
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
#define RUDDER_PIN 6
//SAIL
#define WINCH_PIN 7

//CONFIG//
/////////

//RUDDER
#define RUDDER_POS_MIN  45//47,
#define RUDDER_POS_NEUTRAL  90//94,
#define RUDDER_POS_MAX  135//147,
#define RUDDER_PWM_MIN  553  // according to the seller
#define RUDDER_PWM_MAX  2450  // according to the seller

// these values are inverted on purpose:
#define RUDDER_MIN  -45
#define RUDDER_NEUTRAL  0
#define RUDDER_MAX  45

//SAIL
#define SAIL_MIN 0
#define SAIL_MAX 90
#define SAIL_NEUTRAL SAIL_MIN
#define WINCH_ANGLE_MIN 0 // Physical limit of the servomotor
#define WINCH_ANGLE_MAX 2430  // Physical limit of the servomotor
#define WINCH_PWM_MIN 787  // according to the seller
#define WINCH_PWM_MAX 2173  // according to the seller
#define WINCH_DIAMETER 23 // in mm

#define D_MAST_MAINSAIL_SHEET  390 // between 170 and 710  // Place where the sheet is attached
#define D_MAST_RING 310
#define D_RING_ROPE 20
#define D_WINCH_BOOM 70
#define ROPE_MAX 554
#define ROPE_MIN 0


/*MISC*/
/******/
#define RC_1 0   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_3 2   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_5 4   // Pin 12 Connected to Channel-5 of Transmitter
//PIN//
//////
//LCD I2C
#define LOGGER 0
#define LCD_SCREEN_ADDRESS	0x3f
//RC
#define RC_PIN_1 8   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_PIN_3 10   // Pin 11 Connected to Channel-3 of Transmitter
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