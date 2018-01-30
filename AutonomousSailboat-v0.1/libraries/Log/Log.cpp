#include <Log.h>
#include <Time.h>
// * 
// * @file
// * @brief Feedback Source file
// *
// * Messages
// *   - Intermediates functions to send feedback (user informations / debugging option / logs)
// *   - When writing on the SD card, the first column of the data file indicates the logger 
// *     level (it contains 4 level from the highest (3) to the lowest (0)).
// *   
// *   - Log function:    Log(level, field1, field2);
// *      + Serial: "level"    "field1"    "field2"
// * 
// *      + SD: "level"    "field2"
// * 
// *      + LCD: Nothing
// * 
// *      + LED: Nothing
// *   
// *   - Message function:    Message(type, field1, field2, led);
// *      + Serial: 3    "type"    "field1"    "field2"
// * 
// *      + SD: 3    "field1"    "field2"
// *                  
// *      + LCD Screen: "field1"
// *                  "field2"
// *                
// *      + LED: 0 if led = 0
// *           1 if led = 1
// *   
// *   - Warning function:    Warning(function, message);
// *      + Serial/SD: 3    __________ WARNING ___________
// *                   3    Function Name: "function"
// *                   3    Message: "message"
// *                   3    ______________________________
// *                  
// *      + LCD Screen: Nothing
// *                
// *      + LED: Nothing
// *   
// *   - Error function:    Error(function, message);
// *      + Serial/SD: 3    =========== ERROR ============
// *                   3    Function Name: "function"
// *                   3    Message: "message"
// *                   3    ==============================
// *                  
// *      + LCD Screen: ==== ERROR ====
// *                  "function"
// *                
// *      + LED: 10 x fast blink

Logger* Logger::instance = 0;


// // ####################################################
// // #                                                  #
// // #                    VARIABLES                     #
// // #                                                  #
// // ####################################################
// // LED_BUILTIN:
// #ifndef LED_BUILTIN
// // older versions of the software:
// const int LED_BUILTIN = 13;
// #endif


// // SD card:
// #ifdef SD_ACTIVATED
// File MySDFile;
// Sd2Card SDcard;
// SdVolume SDvolume;

// // Name of the file on the SD card:
// const char* SDFileName = "", //"LOG_000";  // empty string to activate automatic naming otherwise, put a name <= 8 characters
// *p_SDFileName = &SDFileName,  // associated pointer
// buffHead = "",  // Buffer for the logs: level 2
// *p_buffHead = &buffHead,  // associated pointer
// buff = "",  // Buffer for the logs: level 2
// *p_buff = &buff;  // associated pointer

// int SD_Init = 1,
// *p_SD_Init = &SD_Init;  // associated pointer
// #endif

// // RTC module:
// #ifdef RTC_ACTIVATED
// struct ts time;
// #endif

// // Time to read messages:
// const unsigned int TIME_TO_READ_MESSAGE = 1000;

// #ifdef LCD_SCREEN_ACTIVATED
// LiquidCrystal_I2C Logger::lcd = LiquidCrystal_I2C(0x27, 16, 2);
// #endif


// // ####################################################
// // #                                                  #
// // #                    FUNCTIONS                     #
// // #                                                  #
// // ####################################################
// #ifdef RTC_ACTIVATED
// //  ====================================================
// //  =                                                  =
// //  =                    RTC INIT                      =
// //  =                                                  =
// //  ====================================================
// void RTCInit() {
// // Initialization RTC module:
// DS3231_init(DS3231_INTCN);
// }
// #endif


// //  ====================================================
// //  =                                                  =
// //  =                  MESSAGES SETUP                  =
// //  =                                                  =
// //  ====================================================
// void Logger::MessagesSetup() {
// // RTC module: ---------------------------------
// #ifdef RTC_ACTIVATED
// RTCInit();
// #endif


// // LED setup: ----------------------------------
// #ifdef LED_ACTIVATED
// // Safety:
// #ifndef LED_BUILTIN
// // Generation of a compiler error:
// #error "LED_BUILTIN" PIN NOT DECLARED! - THIS MAY BE DUE TO AN OLD VERSION OF THE COMPILER - ADD IT TO THE "Wiring.h" FILE
// #endif
// pinMode(LED_BUILTIN, OUTPUT);
// digitalWrite(LED_BUILTIN, LOW);
// #endif


// // Serial port setup: --------------------------
// #ifdef SERIAL_ACTIVATED
// Serial.begin(SERIAL_BAUD_LOG, SERIAL_8N1);

// // Waiting for initialization:
// while (!Serial) {
// ; // wait for serial port to connect. Needed for native USB
// }
// #endif


// // LCD Setup: ----------------------------------
// #ifdef LCD_SCREEN_ACTIVATED
// lcd.begin();
// #endif
// }


// #ifdef SD_ACTIVATED
// //  ====================================================
// //  =                                                  =
// //  =                     SD SETUP                     =
// //  =                                                  =
// //  ====================================================
// void Logger::SDSetup() {
// // Safety:
// #ifndef SD_PIN_CHIP_SELECT
// // Generation of a compiler error:
// #error "SD_PIN_CHIP_SELECT" NOT DECLARED! see file "Wiring.h"
// #endif
// //pinMode(SD_PIN_CHIP_SELECT, OUTPUT);  // Not necessary

// bool error = false;

// if (!SDcard.init(SPI_HALF_SPEED, SD_PIN_CHIP_SELECT)) {
// Error("SDSetup()", "Initialization failed. Things to check:\n\t\t* is a card inserted?\n\t\t* is your wiring correct?\n\t\t* did you change the chipSelect pin to match your shield or module?");
// error = true;
// }

// // Checks File-system:
// if (!SDvolume.init(SDcard)) {
// Error("SDSetup()", "Could not find FAT16/FAT32 partition on the SD card.\n\t\tMake sure you've formatted the card");
// error = true;
// }

// if (!SD.begin(SD_PIN_CHIP_SELECT)) {
// Error("SDSetup()", "SD card Initialization failed! - No SD card");
// error = true;
// }

// // Test quickly that we can write / read data on the SD card: 
// // TODO:
// // I haven't successfully founded an automated way to do it, because the way to read 
// // data has int as an output (ascii) but not character and the way to read is not really linear.
// const char* TestFile = "TEST.TXT";

// // End of the test: 
// // Be sure to suppress all remaining files of this test
// if (SD.exists(TestFile)) {
// SD.remove(TestFile);
// }

// // Generating the file on which we will write:
// if (!error) {
// // File Naming:
// SDGenerateFileName(p_SDFileName);

// // Be sure that the file is unique:
// if (SD.exists(*p_SDFileName)) {
// SD.remove(*p_SDFileName);
// }
// }
// }


// //  ====================================================
// //  =                                                  =
// //  =               GENERATE NAME FILE                 =
// //  =                                                  =
// //  ====================================================
// void Logger::SDGenerateFileName(const char *p_SDFileName) {
// String extension = ".TXT";  // Frame extension

// int len = 0;

// len = SDFileName.length();
// len += extension.length();
// //*p_SDFileName += "/Logs/";  // Location, which can't be used without reducing the size of the name
// if (len == 4) {
// #ifdef RTC_ACTIVATED
// String wrongBuff = "25165165";  // Wrong frame

// const int BUFFER_SIZE = 9;
// char buff[BUFFER_SIZE];  // Temporary variable

// // Generation of the name of the file (using the time UTC):
// DS3231_get(&time);  // catching data of the DS3231

// // Generating the frame:
// snprintf(buff,
// BUFFER_SIZE,
// "%02d%02d%02d%02d",
// time.mon,
// time.mday,
// time.hour,
// time.min);

// if (wrongBuff.equals(String(buff))) {
// Warning("SDGenerateFileName()", "RTC overpower: You need to plug the Vcc pin from the RTC to 3.3V!!!");
// // See: https://forum.arduino.cc/index.php?topic=189283.0

// // Changing filename:
// *p_SDFileName = "LOG";
// }

// // Adding the extension:
// *p_SDFileName = buff + extension;
// #else
// // Generating a unique file-name:
// int k = 0;
// *p_SDFileName = "LOG";
// while (SD.exists(SDFileName + String(k) + extension)) {
// k ++;
// }
// *p_SDFileName += String(k) + extension;
// #endif
// }
// else if ((len > 4) && (len <= 12)) {  // must respect 8.3 format (8 characters, a dot and 3 characters for the extension)
// // We keep the initial name:
// *p_SDFileName += extension;
// }

// if (SDFileName.length() > 12) {  // In case that the name does not fit the requirements:
// // A default name is assigned:
// Warning("GenerateFileName", "FileName to long - Impossible to save data on the SD card, a new name will be chosen");
// *p_SDFileName = "LOG.TXT";  // In case that the new name is to long
// }
// }


// //  ====================================================
// //  =                                                  =
// //  =                      SAVE SD                     =
// //  =                                                  =
// //  ====================================================
// void Logger::SaveSD() {
// Log(1, F("SaveSD()"), F(""));
// // Open the file:
// MySDFile = SD.open(SDFileName, FILE_WRITE);

// if (MySDFile) {  // writing in a secure way
// if (*p_SD_Init) {
// #ifdef RTC_ACTIVATED
// *p_buffHead += "\t";
// *p_buffHead += "Time:";
// #endif
// // Writing data header:
// MySDFile.print("2\t\t\t");
// MySDFile.println(buffHead);

// // End of the generation of the header:
// *p_SD_Init = 0;
// }
// #ifdef RTC_ACTIVATED
// // Adding time to the logs:
// DS3231_get(&time);
// *p_buff += "\t";
// *p_buff += time.hour;
// *p_buff +=  ":";
// *p_buff += time.min;
// #endif

// // Writing data from the buffer:
// MySDFile.print("2\t\t\t");
// MySDFile.println(*p_buff);

// // Emptying buffer
// *p_buff = "";  

// // Saves data, in case of a problem:
// MySDFile.close();
// }
// }
// #endif


// //  ====================================================
// //  =                                                  =
// //  =                       LOG                        =
// //  =                                                  =
// //  ====================================================
// void Logger::Log(int level, String field1, String field2) {
// // Serial port: --------------------------
// #ifdef SERIAL_ACTIVATED
// Serial.print(level);
// switch (level) {
// case 2:
// Serial.print(F("\t\t\t\t"));
// break;
// case 1:
// Serial.print(F("\t\t\t\t\t\t\t\t"));
// break;
// case 0:
// Serial.print(F("\t\t\t\t\t\t\t\t\t\t\t\t"));
// break;
// default:
// Error("Log", "Unknown log level !");
// break;
// }
// Serial.print(field1);
// Serial.print(F("\t"));
// Serial.println(field2);
// #endif


// // SD card: ------------------------------
// #ifdef SD_ACTIVATED
// // write inside of the appropriate buffer, data requested
// switch (level) {
// case 2:  // log high level
// if (SD_Init) {  // Adding the name of the data kind to the head of the file
// *p_buffHead += field1;
// *p_buffHead += "\t\t";
// }
// *p_buff += field2;
// *p_buff += "\t\t";
// break;
// case 1:  // log medium level
// // Indicating the name of the functions in field1, other informations in field 2

// MySDFile = SD.open(SDFileName, FILE_WRITE);  // Open the file

// if (MySDFile) {  // writing in a secure way
// // Writing data:
// MySDFile.print(level);
// MySDFile.print("\t\t\t\t\t\t\t\t");
// MySDFile.print(field1);
// MySDFile.print(" ");
// MySDFile.println(field2);

// // Saves data, in case of a problem:
// MySDFile.close();
// }
// break;
// case 0:  // log low level
// // Indicating the loop in which the code is located in field1, other informations in field 2

// MySDFile = SD.open(SDFileName, FILE_WRITE);  // Open the file

// if (MySDFile) {  // writing in a secure way
// // Writing data:
// MySDFile.print(level);
// MySDFile.print("\t\t\t\t\t\t\t\t\t\t\t\t");
// MySDFile.print(field1);
// MySDFile.print(" ");
// MySDFile.println(field2);

// // Saves data, in case of a problem:
// MySDFile.close();
// }
// break;
// default:
// Error("Log", "Unknown log level !");
// break;
// }
// #endif


// // LCD Screen: ---------------------------
// //#ifdef LCD_SCREEN_ACTIVATED
// // No action
// //#endif


// // LED: ----------------------------------
// //#ifdef LED_ACTIVATED
// // No action
// //#endif
// }


// //  ====================================================
// //  =                                                  =
// //  =                      MESSAGE                     =
// //  =                                                  =
// //  ====================================================
// void Logger::Message(String type, String field1, String field2, int led) {
// // Serial port: --------------------------
// #ifdef SERIAL_ACTIVATED
// Serial.print(F("3\t"));
// Serial.print(type);
// Serial.print(F("\t"));
// Serial.print(field1);
// Serial.print(F(" "));
// Serial.println(field2);
// #endif


// // SD card: ------------------------------
// #ifdef SD_ACTIVATED
// MySDFile = SD.open(SDFileName, FILE_WRITE);

// if (MySDFile) {  // writing in a secure way
// // Writing Message:
// MySDFile.print("3");
// MySDFile.print(F("\t"));
// MySDFile.print(field1);
// MySDFile.print(F("\t"));
// MySDFile.println(field2);

// // Saves log, in case of a problem:
// MySDFile.close();
// }
// #endif


// // LCD Screen: ---------------------------
// #ifdef LCD_SCREEN_ACTIVATED
// lcd.clear();
// lcd.setCursor(0,0); // Not necessary
// lcd.print(field1);
// lcd.setCursor(0,1);
// lcd.print(field2);
// #endif


// // LED: ----------------------------------
// #ifdef LED_ACTIVATED
// if (led == 1) {
// digitalWrite(LED_BUILTIN, HIGH);
// }
// else {
// digitalWrite(LED_BUILTIN, LOW);
// }
// #endif
// }


// //  ====================================================
// //  =                                                  =
// //  =                      WARNING                     =
// //  =                                                  =
// //  ====================================================
// void Logger::Warning(String function, String message) {
// // Serial port: --------------------------
// #ifdef SERIAL_ACTIVATED
// Serial.println(F("3\n3 __________ WARNING ___________"));
// Serial.print(F("Function Name:\t"));
// Serial.println(function);
// Serial.print(F("3 Message:\t"));
// Serial.println(message);
// Serial.println(F("3 ______________________________\n3"));
// #endif

// // SD card: ------------------------------
// #ifdef SD_ACTIVATED
// MySDFile = SD.open(SDFileName, FILE_WRITE);

// if (MySDFile) {  // writing in a secure way
// // Writing warning Message:
// MySDFile.println(F("3\t\n3\t__________ WARNING ___________"));
// MySDFile.print(F("3\tFunction Name:\t"));
// MySDFile.println(function);
// MySDFile.print(F("3\tMessage:\t"));
// MySDFile.println(message);
// MySDFile.println(F("3\t______________________________\n3"));

// // Saves message, in case of a problem:
// MySDFile.close();
// }
// #endif


// // LCD Screen: ---------------------------
// //#ifdef LCD_SCREEN_ACTIVATED
// // I've chosen not to print information on the LCD screen because a warning is not something critical.
// //#endif


// // LED: ----------------------------------
// #ifdef LED_ACTIVATED
// // Blink 4 times:
// for (int i = 0 ; i < 4 ; i ++) {
// digitalWrite(LED_BUILTIN, LOW);
// delay(500);
// digitalWrite(LED_BUILTIN, HIGH);
// delay(500);    
// }
// #endif

// delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
// }


// //  ====================================================
// //  =                                                  =
// //  =                      ERROR                       =
// //  =                                                  =
// //  ====================================================
// void Logger::Error(String function, String message) {
// // Serial port: --------------------------
// #ifdef SERIAL_ACTIVATED  
// Serial.println(F("3\n3 =========== ERROR ============"));
// Serial.print(F("3 Function Name:\t"));
// Serial.println(function);
// Serial.print(F("3 Message:\t"));
// Serial.println(message);
// Serial.println(F("3 ==============================\n3"));
// #endif

// // SD card: ------------------------------
// #ifdef SD_ACTIVATED
// MySDFile = SD.open(SDFileName, FILE_WRITE);

// if (MySDFile) {  // writing in a secure way
// // Writing error Message:
// MySDFile.println(F("3\t\n3\t=========== ERROR ============"));
// MySDFile.print(F("3\tFunction Name:\t"));
// MySDFile.println(function);
// MySDFile.print(F("3\tMessage:\t"));
// MySDFile.println(message);
// MySDFile.println(F("3\t==============================\n3"));

// // Saves message, in case of a problem:
// MySDFile.close();
// }
// #endif


// // LCD Screen : --------------------------
// #ifdef LCD_SCREEN_ACTIVATED
// lcd.clear();
// lcd.setCursor(0,0);  // Not necessary
// lcd.print(F("==== ERROR ===="));
// lcd.setCursor(0,1);
// lcd.print(function);
// #endif


// // LED: ----------------------------------
// #ifdef LED_ACTIVATED
// int i = 0;  // LED counter

// // Blink 4 times slowly, then 10 quickly:
// for (int i = 0 ; i < 4 ; i ++) {
// digitalWrite(LED_BUILTIN, LOW);
// delay(500);
// digitalWrite(LED_BUILTIN, HIGH);
// delay(500);    
// }
// for (i = 0; i < 10; i ++) {
// digitalWrite(LED_BUILTIN, HIGH);
// delay(300);
// digitalWrite(LED_BUILTIN, LOW);
// delay(300);
// }
// #endif

// Message(F("# RESET #"), F("The Arduino board will be reset in 5 seconds"), F(""), 0);
// delay(5*TIME_TO_READ_MESSAGE);  // Time to read because it is important
// ResetArduino();
// }
void Logger::MessagesSetup(){
	lcd.begin(16,2);
	lcd.clear();
	lcd.home();
}

void Logger::printLCD(String s1, String s2){
	lcd.clear();
	scrollSize = 0;
	scrollSizeActual = 0;
	
	int max = s1.length();
	if(s2.length() > max)
		max = s2.length();
	
	
	lcd.setCursor(0,0);
	lcd.print(s1);
	lcd.setCursor(0,1);
	lcd.print(s2);
	
	if(max > 16){
		scrollSize = max-16;
		timer = millis();
	}
	//lcd.home();
}
void Logger::Log(int level, String field1, String field2){
	printLCD(field1, field2);
}

void Logger::Message(String type, String field1, String field2, int led){
	printLCD(field1, field2);
}

void Logger::Warning(String function, String message){
	printLCD(function, message);
}

void Logger::Error(String function, String message){
	printLCD(function, message);
}

void Logger::Toast(String field1, String field2, unsigned long ms){
	if(ms == 0){
		prevM1 = field1;
		prevM2 = field2;
	}
	printLCD(field1, field2);
	timerToast = millis();
	ToastMS = ms;
	printing = true;
}

void Logger::Update(){
	if(scrollSize > 0){
		if(scrollSizeActual == 0 && millis() - timer > 2000){
			scrollSizeActual++;
			lcd.scrollDisplayLeft();
			timer = millis();
		}
		if(scrollSizeActual < scrollSize && scrollSizeActual > 0 && millis() - timer > 450){
			scrollSizeActual++;
			lcd.scrollDisplayLeft();
			timer = millis();
		}
		if(scrollSizeActual >= scrollSize && millis() - timer > 1000){
			scrollSizeActual = 0;
			lcd.home();
			timer = millis();
		}
	}
	if(printing){
		if(millis() - timerToast > ToastMS){
			printLCD(prevM1, prevM2);
			printing = false;
		}
	}
}