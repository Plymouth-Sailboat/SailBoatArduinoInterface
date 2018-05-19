#ifndef CONFIG_H
#define CONFIG_H

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
//WIND
#define WIND_SENSOR_MIN 48  // Corresponds approximately at 5% of the maximum readable (1023)
#define WIND_SENSOR_MAX 990  // Corresponds approximately at 95% of the minimum readable (1023)
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
#define RUDDER_POS_MIN  68//47,
#define RUDDER_POS_NEUTRAL  103//94,
#define RUDDER_POS_MAX  150//147,
//#define RUDDER_PWM_MIN  553  // according to the seller
//#define RUDDER_PWM_MAX  2450  // according to the seller

// these values are inverted on purpose:
#define RUDDER_MIN  -45
#define RUDDER_NEUTRAL  0
#define RUDDER_MAX  45

//SAIL
#define SAIL_MIN 0
#define SAIL_NEUTRAL SAIL_MIN
#define SAIL_MAX 90

#define WINCH_ANGLE_MIN 38 // Physical limit of the servomotor
#define WINCH_ANGLE_NEUTRAL WINCH_ANGLE_MIN // Physical limit of the servomotor
#define WINCH_ANGLE_MAX 112  // Physical limit of the servomotor
//#define WINCH_PWM_MIN 24  // according to the seller
//#define WINCH_PWM_MAX 2173  // according to the seller
#define WINCH_DIAMETER 23 // in mm

/*#define D_MAST_MAINSAIL_SHEET  390 // between 170 and 710  // Place where the sheet is attached
#define D_MAST_RING 310
#define D_RING_ROPE 20
#define D_WINCH_BOOM 70
#define ROPE_MAX 554
#define ROPE_MIN 0*/


/*MISC*/
/******/
//PIN//
//////
//LCD I2C
#define LOGGER 0
#define LCD_SCREEN_ADDRESS	0x3f
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
