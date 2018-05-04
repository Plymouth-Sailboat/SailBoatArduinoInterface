#ifndef LOG_H
#define LOG_H

#include <string.h>
#include <Wire.h>
#include <Helper.h>
#include <config-Sailboat.h>

#include <ros.h>
#include <std_msgs/Header.h>

//Real Time Clock:
#ifdef RTC_ACTIVATED
#include "ds3231.h"
#endif

// SD card:
#ifdef SD_ACTIVATED
#include <SPI.h>
#include <SD.h>
#endif

#include <LiquidCrystal_I2C.h> 

#ifdef SERIAL_ACTIVATED
#define SERIAL_BAUD_LOG 9600
#endif

#include <Arduino.h>
#include <config.h>

class Logger{
public:
	Logger() : lcd(LCD_SCREEN_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE), scrollSizeActual(0), scrollSize(0), timer(0), printing(false), prevM1(" "), prevM2(" "){}
	
	void MessagesSetup();
	/**
* Logging Function
* Prints out message on the SD card and the serial port - this function is made for debug
*
* @param level: Number indicating the level of information - the higher the number is,
*               the more high-level it is (this numbers allows 3 levels : 0, 1, 2)
* @param field1: String containing the message
* @param field2: String containing the message
*
* @return Serial: "level"    "field1"    "field2"
*
* @return SD: "level"    "field2"
*
* @return LCD: Nothing
*
* @return LED: Nothing
*/
	void Log(int level, String field1, String field2);


	/**
* Message Function
* Prints out message on every output available
*
* @param type: String containing the kind of information / will not be printed on the LCD screen
* @param field1: String containing the message / will be printed on the screen
* @param field2: String containing the message / will be printed on the screen
* @param led: integer which trigger the led (1: led on / 0: led off)
*
* @return Serial: 3    "type"    "field1"    "field2"
*
* @return SD: 3    "field1"    "field2"
*
* @return LCD Screen: "field1"
*                     "field2"
*
* @return LED: 0 if led = 0
*              1 if led = 1
*/
	void Message(String type, String field1, String field2, int led);


	/**
* Warning Function
* Prints out error warning on the serial port and SD card
*
* @param function: String containing the name of the function
* @param message: String containing the warning message
*
* @return Serial/SD: 3    __________ WARNING ___________
*                    3    Function Name: "function"
*                    3    Message: "message"
*                    3    ______________________________
*
* @return  LCD Screen: Nothing
*
* @return  LED: Nothing
*/
	void Warning(String function, String message);


	/**
* Error Function
* Prints out error message on every output available
*
* @param function: String containing the name of the function
* @param message: String containing the error message
*
* @return Serial/SD: 3    =========== ERROR ============
*                    3    Function Name: "function"
*                    3    Message: "message"
*                    3    ==============================
*
* @return LCD Screen: ==== ERROR ====
*                     "function"
*
* @return  LED: 10 x fast blink
*/
	void Error(String function, String message);
	
	void Toast(String field1, String field2, unsigned long ms);
	
	void Update();
	static Logger* Instance(){if(!instance)instance = new Logger(); return instance;}
private:
	static Logger* instance;
	
	LiquidCrystal_I2C lcd;
	void printLCD(String s1, String s2);

	
	int scrollSize;
	int scrollSizeActual;
	unsigned long timer;
	unsigned long timerToast;
	unsigned long ToastMS;
	bool printing;
	String prevM1;
	String prevM2;

#ifdef SD_ACTIVATED
	static void SDSetup();
	static void SDGenerateFileName(String *p_SDFileName);
	static void SaveSD();
	#endif
};

#endif