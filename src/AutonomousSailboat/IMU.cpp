/**
 * @file
 * @brief IMU Source file
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


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "IMU.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
// IMU data container:
IMUStruct IMU;

// Associated pointer:
p_IMUStruct *p_IMU = &IMU;

// Heading variable:
double heading = 0.0f,
       *p_heading = &heading,
       heading1 = 0.0f,
       *p_heading1 = &heading1,
       heading2 = 0.0f,
       *p_heading2 = &heading2;


// Magnetometer calibration parameters:
const double M_CENTER_X = 0.0978f, // translation vector - Compass calibration
             M_CENTER_Y = 47.2903f,
             M_CENTER_Z = 16.4626f,
             
             CC11 = 0.9960f,  // Homothetic Matrix - Compass calibration
             CC12 = 0.0014f,
             CC13 = -0.0124f,
             CC21 = 0.0014f,
             CC22 = 0.9719f,
             CC23 = 0.0029f,
             CC31 = -0.0124f,
             CC32 = 0.0029f,
             CC33 = 0.9612f,
             
             c0 = 1.0f,  // Coefficients of the moving average filter
             c1 = 1.0f,
             c2 = 1.0f;;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                     IMU SETUP                    =
//  =                                                  =
//  ====================================================
void IMUSetup (p_IMUStruct *p_IMU){
  //Log(1, F("IMUSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino

  // Initialization of the data structure:
  p_IMU->p_Accelero->x = 0;
  p_IMU->p_Accelero->y = 0;
  p_IMU->p_Accelero->z = 0;

  p_IMU->p_Gyro->x = 0;
  p_IMU->p_Gyro->y = 0;
  p_IMU->p_Gyro->z = 0;

  p_IMU->p_Magneto->x = 0;
  p_IMU->p_Magneto->y = 0;
  p_IMU->p_Magneto->z = 0;

  // Initialise the I2C bus
  Wire.begin();

  // Initialise the accelerometer and gyro and put the I2C bus into pass-through mode
  Initalise_AccelGyro(ACCELRANGE_8g, GYRORANGE_2000DPS);
}


//  ====================================================
//  =                                                  =
//  =                      IMU LOOP                    =
//  =                                                  =
//  ====================================================
void IMULoop(p_IMUStruct *p_IMU, double *p_heading) {
  Log(1, F("IMULoop()"), F(""));

  // Temporary variables:
  int Mx = 0,
      My = 0,
      Mz = 0;
  double Mxc = 0.0f,
         Myc = 0.0f,
         Mzc = 0.0f;

  // Trigger a compass measurement:
  Trigger_Compass();
  
  // Read the compass X, Y, and Z axis
  Mx = Read_Compass(COMP_XOUT_L);
  My = Read_Compass(COMP_YOUT_L);
  Mz = Read_Compass(COMP_ZOUT_L);

  // Applying Calibration:
  //_______
  // Homothety: Matrix product
  Myc = Mx*CC11 + My*CC12 + Mz*CC13;
  Mxc = Mx*CC21 + My*CC22 + Mz*CC23;
  Mzc = -Mx*CC31 - My*CC32 - Mz*CC33;

  // Translation: Vector substraction:
  Mxc -= M_CENTER_X;  // in fact Xmagneto = Yaccelero
  Mxc -= M_CENTER_Y;  // in fact Ymagneto = Xaccelero
  Mzc -= M_CENTER_Z;  // in fact Zmagneto = - Zaccelero
  //_________________________________________

  // Stores the new values into the structure:
  p_IMU->p_Magneto->x = Mxc,
  p_IMU->p_Magneto->y = Myc,
  p_IMU->p_Magneto->z = Mzc;
  

  // Finding the heading:
  *p_heading = findHeading(IMU.Magneto.x, IMU.Magneto.y);
}


//  ====================================================
//  =                                                  =
//  =               INITIALIZE ACCEL GYRO              =
//  =                                                  =
//  ====================================================
void Initalise_AccelGyro(byte Accel_Range, byte Gyro_Range) {
  /* Take the MPU9150 out of sleep */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(PWR_MGMT_1); 
  Wire.write(0); 
  Wire.endTransmission(); 
  
  /* Set the sensitivity of the module */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(ACCEL_CONFIG); 
  Wire.write(Accel_Range << 3); 
  Wire.endTransmission(); 
  
  /* Set the sensitivity of the module */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(GYRO_CONFIG); 
  Wire.write(Gyro_Range << 3); 
  Wire.endTransmission(); 
  
  /* Put the I2C bus into pass-through mode so that the aux I2C interface
     that has the compass connected to it can be accessed */
  Wire.beginTransmission(I2CACCGYROADD); 
  Wire.write(0x6A); 
  Wire.write(0x00); 
  Wire.endTransmission(true);

  Wire.beginTransmission(I2CACCGYROADD); 
  Wire.write(0x37); 
  Wire.write(0x02); 
  Wire.endTransmission(true); 
}


//  ====================================================
//  =                                                  =
//  =                   READ COMPASS                   =
//  =                                                  =
//  ====================================================
int Read_Compass(byte axis) {
  Log(1, F("Read_Compass()"), F(""));

  int Data;
 
  // Select the required axis register
  Wire.beginTransmission(I2CCOMPADD); 
  Wire.write(axis); 
  Wire.endTransmission(); 
 
  // Request the low and high bytes for the required axis
  Wire.requestFrom(I2CCOMPADD, 2);
  Data = Wire.read();
  Data = Data | (int)(Wire.read() << 8);
  Wire.endTransmission(); 
  
  return Data;
}


//  ====================================================
//  =                                                  =
//  =                  TRIGGER COMPASS                 =
//  =                                                  =
//  ====================================================
void Trigger_Compass(void) {
  Log(1, F("Trigger_Compass()"), F(""));

  // Trigger a measurement
  Wire.beginTransmission(I2CCOMPADD); 
  Wire.write(0x0A); 
  Wire.write(0x01); 
  Wire.endTransmission(true);
  
  // Wait for the measurement to complete
  do {
    Wire.beginTransmission(I2CCOMPADD); 
    Wire.write(COMP_STATUS); 
    Wire.endTransmission(); 
 
    Wire.requestFrom(I2CCOMPADD, 1);
  }while(!Wire.read()); 
}


//  ====================================================
//  =                                                  =
//  =                   FIND HEADING                   =
//  =                                                  =
//  ====================================================
double findHeading(int compass_x, int compass_y) {
  Log(1, F("findHeading()"), F(""));
  // TODO: Implement a tilt compensated compass

  double angle,
         compass_x1,
         compass_y1;

  // Compatibility:
  compass_x1 = -(double)compass_y;
  compass_y1 = (double)compass_x;

  // Case that the IMU is laying flat:
  if (compass_y1 == 0) {
    angle = heading;
  }
  else {

    // New angle:
    angle = RAD_TO_DEG*atan2(compass_x1, compass_y1);  // We follow only data on the absolute plane XY

    // Moving average filter:
    *p_heading2 = heading1;
    *p_heading1 = heading;
    angle = (c2*heading2 + c1*heading1 + c0*angle)/(c0 + c1 + c2);
  }
  return (angle);
}