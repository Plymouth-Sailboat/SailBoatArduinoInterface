#include <IMU.h>
#include <Wire.h>

void IMU::init(ros::NodeHandle* n){
	Wire.begin();
	
	Wire.beginTransmission(I2CACCGYROADD);
	Wire.write(PWR_MGMT_1); 
	Wire.write(0); 
	Wire.endTransmission(); 

	/* Set the sensitivity of the module */
	Wire.beginTransmission(I2CACCGYROADD);
	Wire.write(ACCEL_CONFIG); 
	Wire.write(ACCELRANGE_8g << 3); 
	Wire.endTransmission(); 

	/* Set the sensitivity of the module */
	Wire.beginTransmission(I2CACCGYROADD);
	Wire.write(GYRO_CONFIG); 
	Wire.write(GYRORANGE_2000DPS << 3); 
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
	

	SensorROS::init(n);
}

void IMU::updateMeasures(){
	//	Logger::Log(1, F("IMULoop()"), F(""));

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
	IMUs.Magneto.x = Mxc,
	IMUs.Magneto.y = Myc,
	IMUs.Magneto.z = Mzc;


	// Finding the heading:
	heading = findHeading(IMUs.Magneto.x, IMUs.Magneto.y);
}
void IMU::updateTest(){
	heading = 1;
}

int IMU::Read_Compass(byte axis) {
	//	Logger::Log(1, F("Read_Compass()"), F(""));

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

void IMU::Trigger_Compass(void) {
	//	Logger::Log(1, F("Trigger_Compass()"), F(""));

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

double IMU::findHeading(int compass_x, int compass_y) {
	//	Logger::Log(1, F("findHeading()"), F(""));
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
		heading2 = heading1;
		heading1 = heading;
		angle = (C2*heading2 + C1*heading1 + C0*angle)/(C0 + C1 + C2);
	}
	return (angle);
}

void IMU::communicateData(){
	msg.orientation.x = 0.980;
	msg.orientation.y = 0.001;
	msg.orientation.z = 0.189;
	msg.orientation.w = -0.063;
	
	msg.header.stamp = nh->now();
	pub.publish(&msg);
}
