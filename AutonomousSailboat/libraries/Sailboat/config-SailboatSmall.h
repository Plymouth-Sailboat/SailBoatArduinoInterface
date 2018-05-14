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
#define RC_1 0   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_2 1   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_3 2   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_4 3   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_5 4   // Pin 12 Connected to Channel-5 of Transmitter
#define RC_6 5   // Pin 12 Connected to Channel-5 of Transmitter
//PIN//
//////
//LCD I2C
#define LOGGER 0
#define LCD_SCREEN_ADDRESS	0x3f
//RC
#define RC_PIN_1 8   // Pin 10 Connected to Channel-1 of Transmitter
#define RC_PIN_2 9   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_PIN_3 10   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_PIN_4 11   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_PIN_5 12   // Pin 12 Connected to Channel-5 of Transmitter
#define RC_PIN_6 13   // Pin 11 Connected to Channel-3 of Transmitter
#define RC_NUM_CHANNELS 6
//RC Config
#define RC_1_MIN	1012
#define RC_1_MAX	1924
#define RC_2_MIN	1008
#define RC_2_MAX	1916
#define RC_3_MIN	1332
#define RC_3_MAX	1902
#define RC_4_MIN	912
#define RC_4_MAX	2050
#define RC_5_MIN	0
#define RC_5_MAX	0
#define RC_6_MIN	0
#define RC_6_MAX	0
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
