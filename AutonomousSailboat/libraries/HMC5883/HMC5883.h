#ifndef HMC5883_SENSOR_H
#define HMC5883_SENSOR_H

#include <SensorsInterface.h>
#include <sensor_msgs/MagneticField.h>


#define SENSORS_GAUSS_TO_MICROTESLA (100) /**< Gauss to micro-Tesla multiplier */
#define SENSORS_GAUSS_TO_TESLA (0.0001) /**< Gauss to micro-Tesla multiplier */
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define HMC5883_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_REGISTER_MAG_CRA_REG_M             = 0x00,
      HMC5883_REGISTER_MAG_CRB_REG_M             = 0x01,
      HMC5883_REGISTER_MAG_MR_REG_M              = 0x02,
      HMC5883_REGISTER_MAG_OUT_X_H_M             = 0x03,
      HMC5883_REGISTER_MAG_OUT_X_L_M             = 0x04,
      HMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      HMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      HMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      HMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      HMC5883_REGISTER_MAG_SR_REG_Mg             = 0x09,
      HMC5883_REGISTER_MAG_IRA_REG_M             = 0x0A,
      HMC5883_REGISTER_MAG_IRB_REG_M             = 0x0B,
      HMC5883_REGISTER_MAG_IRC_REG_M             = 0x0C,
      HMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      HMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } hmc5883MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } hmc5883MagGain;	
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct hmc5883MagData_s
    {
        float x;
        float y;
        float z;
      float orientation;
    } hmc5883MagData;
			 
class HMC5883 : public SensorROS{
	public:
		HMC5883() : SensorROS("compass", &msg),X(0),Y(0),Z(0){}
		void init(ros::NodeHandle* n);
		void updateMeasures();
		void updateTest();
		void communicateData();
		
		void setMagGain(hmc5883MagGain gain);
		
	private:
		hmc5883MagGain   _magGain;
		hmc5883MagData   _magData;     // Last read magnetometer data will be available here
		sensor_msgs::MagneticField msg;
		double X,Y,Z;
		float heading;
		
		void write8(byte address, byte reg, byte value);
		byte read8(byte address, byte reg);
		void read(void);
};

#endif
