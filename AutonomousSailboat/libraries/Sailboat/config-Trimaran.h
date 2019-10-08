#ifndef CONFIG_H
#define CONFIG_H


/**********************************************ID********************************************/
/**SENSORS**/
/***********/
#define NB_SENSORS		4

#define SENSOR_WINDSENSOR	0
#define SENSOR_GPS			1
#define SENSOR_IMU			2
#define SENSOR_BATTERY      3

#define NB_SENSORS_NOT_ROS		1

#define SENSOR_RC			0

/**ACTUATORS**/
/***********/
#define NB_ACTUATORS		2

#define ACTUATOR_RUDDER		0
#define ACTUATOR_SAIL		1

/**********************************************PINS********************************************/

//RC Transceiver
#define HK_TR6

//Using Grove Shield or not
#define SERVO_SHIELD

//IMU
//#define CMPS12_IMU
#define XSENS_IMU
//#define USE_ARDUINO_GPS
//#define USE_ARDUINO_WIND
#define FUSE_GPS_IMU


/*****************************************CONFIGURATION****************************************/
/**CONFIG**/
/**********/
//WIND
#define WIND_SENSOR_MIN 48
#define WIND_SENSOR_MAX 990

//RUDDER
#define RUDDER_POS_MIN  20
#define RUDDER_POS_NEUTRAL  80
#define RUDDER_POS_MAX  150

#define RUDDER_MIN  -45
#define RUDDER_NEUTRAL  0
#define RUDDER_MAX  45

//SAIL
#define SAIL_MIN 0
#define SAIL_NEUTRAL SAIL_MIN
#define SAIL_MAX 90

#define WINCH_ANGLE_MIN 38
#define WINCH_ANGLE_NEUTRAL WINCH_ANGLE_MIN
#define WINCH_ANGLE_MAX 90
#define WINCH_DIAMETER 23 // in mm

#endif
