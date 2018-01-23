#include <Xsens.h>
#include <Wire.h>


void XBus::parseData(uint8_t* data, uint8_t datalength){
	if(datalength < 2)
		return;
	/*	Serial.println("parsing");
	for(int i = 0; i < datalength; i++){
		Serial.print(data[i], HEX);
		Serial.print(" ");
	}
		Serial.println(" ");*/
	
	actualMID = data[0];
	if(actualMID == 0x10 || actualMID == 0x20 || actualMID == 0x40 || actualMID == 0x80 || actualMID == 0xC0 || actualMID == 0xE0){
		uint8_t length = data[2];
		switch((uint16_t)data[1] |((uint16_t)data[0]<<8)){
			case (uint16_t)DataID::QUAT:
			for(int j = 0; j < length/4; j++){
				uint8_t tmp[4];
				for(int i = 0; i < 4; i++){
					tmp[i] = data[4-i+2+j*4];
				}
				memcpy(&quat[j], tmp, sizeof(float));
			}
				break;
			case (uint16_t)DataID::ACCEL:
			for(int j = 0; j < length/4; j++){
				uint8_t tmp[4];
				for(int i = 0; i < 4; i++){
					tmp[i] = data[4-i+2+j*4];
				}
				memcpy(&accel[j], tmp, sizeof(float));
			}
				break;
			case (uint16_t)DataID::MAG:
			for(int j = 0; j < length/4; j++){
				uint8_t tmp[4];
				for(int i = 0; i < 4; i++){
					tmp[i] = data[4-i+2+j*4];
				}
				memcpy(&mag[j], tmp, sizeof(float));
			}
				break;
			case (uint16_t)DataID::ROT:
			for(int j = 0; j < length/4; j++){
				uint8_t tmp[4];
				for(int i = 0; i < 4; i++){
					tmp[i] = data[4-i+2+j*4];
				}
				memcpy(&rot[j], tmp, sizeof(float));
			}
				break;
		}
		parseData(data+3+length, datalength - length - 3);
	}else{
		uint8_t length = data[1];
		if(actualMID == (uint8_t)MesID::DATA || actualMID == (uint8_t)MesID::DATA2)
			parseData(data+2, length);
	}
}

void XBus::readPipeStatus(){
	Wire.beginTransmission(address);
	Wire.write(PIPE_STATUS);
	Wire.endTransmission();
	

	Wire.requestFrom(address,(uint8_t)4);
	if(Wire.available()>0) {
		for(int i = 0; i < 4; i++){
			data[i] = Wire.read();
		}
	}
	notificationSize = (uint16_t)data[0] | ((uint16_t)data[1]<<8);
	measurementSize = (uint16_t)data[2] | ((uint16_t)data[3]<<8);
}
void XBus::readPipeNotif(){
	Wire.beginTransmission(address);
	Wire.write(NOTIF_PIPE);
	Wire.endTransmission();
	

	Wire.requestFrom(address,notificationSize);
	if(Wire.available()>0) {
		for(int i = 0; i < notificationSize; ++i){
			datanotif[i] = Wire.read();
		}
	}
}
void XBus::readPipeMeas(){
	Wire.beginTransmission(address);
	Wire.write(MEAS_PIPE);
	Wire.endTransmission();
	
	Wire.requestFrom(address,measurementSize);
	if(Wire.available()>0) {
		for(int i = 0; i < measurementSize; ++i){
			datameas[i] = Wire.read();
		}
	}
}

uint8_t* XBus::buildMessage(MesID MID, uint8_t* data, uint8_t length){
	uint8_t* res = new uint8_t[length+4];
	res[0] = (uint8_t)MID;
	res[1] = length;
	if(length > 0 && data != NULL)
		memcpy(res+2, data, length);
	else
		res[2] = 0;
	uint8_t checksum = 0xFF;
	for(int i = 0; i< length+2; i++)
		checksum -= res[0];
	res[length+3] = checksum;
	
	return res;
}

void XBus::quatToHeading(){
	headingYaw = atan2(2*quat[0]*quat[3]+quat[1]*quat[2], 1-2*(quat[2]*quat[2]+quat[3]*quat[3]));
}

void XBus::read(){
	readPipeStatus();
	readPipeNotif();
	readPipeMeas();
	parseData(datameas, measurementSize);
	quatToHeading();
}