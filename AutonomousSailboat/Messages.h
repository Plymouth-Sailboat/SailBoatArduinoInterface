/** 
 * @file
 * @brief Feedback Header file
 *
 * Messages
 *   - Intermediates functions to send feedback (user informations / debugging option / logs)
 *   - When writing on the SD card, the first column of the data file indicates the logger 
 *     level (it contains 4 level from the highest (3) to the lowest (0)).
 *   
 *   - Log function:    Log(level, field1, field2);
 *      + Serial: "level"    "field1"    "field2"
 * 
 *      + SD: "level"    "field2"
 * 
 *      + LCD: Nothing
 * 
 *      + LED: Nothing
 *   
 *   - Message function:    Message(type, field1, field2, led);
 *      + Serial: 3    "type"    "field1"    "field2"
 * 
 *      + SD: 3    "field1"    "field2"
 *                  
 *      + LCD Screen: "field1"
 *                  "field2"
 *                
 *      + LED: 0 if led = 0
 *           1 if led = 1
 *   
 *   - Warning function:    Warning(function, message);
 *      + Serial/SD: 3    __________ WARNING ___________
 *                   3    Function Name: "function"
 *                   3    Message: "message"
 *                   3    ______________________________
 *                  
 *      + LCD Screen: Nothing
 *                
 *      + LED: Nothing
 *   
 *   - Error function:    Error(function, message);
 *      + Serial/SD: 3    =========== ERROR ============
 *                   3    Function Name: "function"
 *                   3    Message: "message"
 *                   3    ==============================
 *                  
 *      + LCD Screen: ==== ERROR ====
 *                  "function"
 *                
 *      + LED: 10 x fast blink
 */


#ifndef MESSAGES_H
#define MESSAGES_H


// ####################################################
// #                                                  #
// #                      MACROS                      #
// #                                                  #
// ####################################################
// Serial: To use it, let it uncommented, otherwise comment it
#define SERIAL_ACTIVATED

// LCD: To use it, let it uncommented, otherwise comment it
#define LCD_SCREEN_ACTIVATED

// SD card: To use it, let it uncommented, otherwise comment it
#define SD_ACTIVATED

// LED: To use it, let it uncommented, otherwise comment it
#define LED_ACTIVATED

// RTC: To use it, let it uncommented, otherwise comment it
//#define RTC_ACTIVATED


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
// Arduino Headers: To use it with eclipse or processing
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <string.h>
#include <Wire.h>

// Tools
#include "Tools.h"
 
// Wiring map:
#include "Wiring.h"

//Real Time Clock:
#ifdef RTC_ACTIVATED
#include "ds3231.h"
#endif

// SD card:
#ifdef SD_ACTIVATED
#include <SPI.h>
#include <SD.h>
#endif

// LCD Screen:
#ifdef LCD_SCREEN_ACTIVATED
#include <LiquidCrystal_I2C.h>  // inclusion only if the variable is set at 1
#endif


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
#ifdef SD_ACTIVATED
  // Buffer for the logs: level 2
  extern String buffHead,
                buff,
                *p_buffHead,
                *p_buff;

  // Name of the file on the SD card:
  extern String SDFileName;

  extern int SD_Init,
             *p_SD_Init;
#endif

// Time to read messages:
extern const unsigned int TIME_TO_READ_MESSAGE;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
#ifdef RTC_ACTIVATED
/**
 * Initialization of the RTC module (connected via I2C field bus)
 */
void RTCInit();
#endif


/**
 * Setup for messaging channels except SD card
 */
void MessagesSetup();


#ifdef SD_ACTIVATED  // If the SD card is activated:
  /**
   * Setup for SD card, must be used after the use of RTC module
   *
   */
  void SDSetup();

  /**
   * Generates the name of the SD card's file to save logs.
   * The name is created based on the UTC date and time
   *
   * @param *p_SDFileName: pointer containing the empty string (automatic naming using GPS UTC) or a specific name (if no GPS)
   */
  void SDGenerateFileName(String *p_SDFileName);

  /**
   * Save SD saves on the SD card, the buffer's content
   */
  void SaveSD();
#endif


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


#endif
