/**
 * @file 
 * @brief IMU Header file
 *
 * IMU:
 *  - Catches data from the IMU
 *  - Parses it
 *  - Stores the result inside of the IMU data structure (see IMU.h)
 *
 * You need to know that I haven't succeed in doing a tilt compensated compass so it works only on the horizontal plane
 *
 * Warning:
 *   !!! The MPU9250 requires 5V for input voltage and 3.3V for I2C bus, so you need to use a level shifter between 
 *   the Arduino board and the I2C pins of the IMU!
 *   You have to do this because all digital output signals of the Arduino Mega are in 5V
 *
 * TODO: Implement a tilt compensated compass
 * I have used a moving average filter instead of correctly obtaining the angle!
 */


#ifndef IMU_H
#define IMU_H


// ####################################################
// #                                                  #
// #                  REGISTER VALUES                 #
// #                                                  #
// ####################################################
// I2C addresses of Accelerometer/Gyro and Compass
#define I2CACCGYROADD 0x68 
#define I2CCOMPADD 0x0C

// Accelerometer/Gyro register addresses
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define PWR_MGMT_1 0x6B

//Compass register addresses
#define COMP_STATUS 0x02
#define COMP_XOUT_L 0x03
#define COMP_YOUT_L 0x05
#define COMP_ZOUT_L 0x07

// Accelerometer range modes
#define ACCELRANGE_2g 0
#define ACCELRANGE_4g 1
#define ACCELRANGE_8g 2
#define ACCELRANGE_16g 3

// Gyroscope sensitivity
#define GYRORANGE_250DPS 0
#define GYRORANGE_500DPS 1
#define GYRORANGE_1000DPS 2
#define GYRORANGE_2000DPS 3


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
// Arduino Headers: For using it with eclipse or processing
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// Wiring map:
#include "Wiring.h"

// Feedback:
#include "Messages.h"


// Include the standard wire library for I2C
#include <Wire.h>

// Mathematical tools:
#include "Tools.h"


// ####################################################
// #                                                  #
// #                    STRUCTURES                    #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                    IMU DATA                      =
//  =                                                  =
//  ====================================================
/**
 * Content of the Main data type structure
 */
typedef struct IMUData {
  float x,  // coordinate x
        y,  // coordinate y
        z;  // coordinate z
} IMUData, p_IMUData;


//  ====================================================
//  =                                                  =
//  =                    IMU STRUCT                    =
//  =                                                  =
//  ====================================================
/**
 * Main data type structure
 */
typedef struct IMUStruct {
  IMUData Accelero,  // Accelerometer data
          Gyro,  // Gyroscope data
          Magneto;  // Magnetometer data

  // Pointers:
  p_IMUData *p_Accelero = &Accelero,  // Accelerometer pointer
            *p_Gyro = &Gyro,  // Gyroscope pointer
            *p_Magneto = &Magneto;  // Magnetometer pointer

} IMUStruct, p_IMUStruct;


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
// IMU data container:
extern IMUStruct IMU;

// Associated pointer:
extern p_IMUStruct *p_IMU;

// Heading variable:
extern double heading,
              *p_heading;


// ####################################################
// #                                                  #
// #                     FUNCTIONS                    #
// #                                                  #
// ####################################################
/**
 * @name Main functions
 * @{
 */
//  ====================================================
//  =                                                  =
//  =                   MAIN FUNCTIONS                 =
//  =                                                  =
//  ====================================================
// THESE FUNCTIONS ARE WHAT YOU'RE SUPPOSED TO USE.
//  CONSIDER THE FOLLOWING FUNCTIONS AS IF THEY WERE DECLARED AS PUBLIC IN AN 
//  OBJECT ORIENTED LANGUAGE.
/**
 * Setup for the IMU
 *  Initialization of the I2C bus
 *
 * @param p_IMU: pointer to the IMUStruct
 */
void IMUSetup (p_IMUStruct *p_IMU);


/**
 * Gathering data for the IMU and finding the heading
 * 
 * @param p_IMU: pointer to the IMUStruct
 * @param p_heading: pointer to the heading (double)
 */
void IMULoop(p_IMUStruct *p_IMU, double *p_heading);

/**
 * @}
 */



/**
 * @name Sub functions
 * @{
 */
//  ====================================================
//  =                                                  =
//  =                   SUB FUNCTIONS                  =
//  =                                                  =
//  ====================================================
// THESE FUNCTIONS ARE USED BY MAIN FUNCTIONS, SO YOU'RE NOT SUPPOSED TO USE IT.
//  CONSIDER THE FOLLOWING FUNCTIONS AS IF THEY WERE DECLARED AS PRIVATE IN AN 
//  OBJECT ORIENTED LANGUAGE.
/**
 * Initialises the accelerometer and gyro to one of the sensitivity ranges 
 * and puts the I2C bus into pass-through mode 
 *
 * @param Accel_Range: 
 * @param Gyro_Range: 
 */
void Initalise_AccelGyro(byte Accel_Range, byte Gyro_Range);


/**
 * Read one of the compass axis
 *
 * @param axis: byte indicating the desired axis in register addresses
 */
int Read_Compass(byte axis);


/**
 * Trigger a single shot compass reading of all three axis 
 *
 */
void Trigger_Compass(void);


/**
 * Extracting the heading from the compass data
 *  Warning: this function is only working if the IMU stays in the horizontal plane
 * 
 * @param compass_x: value of the compass with reference to the x-axis
 * @param compass_y: value of the compass with reference to the y-axis
 *
 * @return the heading of the boat / angle with reference to the north (double)
 *
 * TODO: Implement a tilt compensated compass
 */
double findHeading(int compass_x, int compass_y);

/**
 * @}
 */


#endif