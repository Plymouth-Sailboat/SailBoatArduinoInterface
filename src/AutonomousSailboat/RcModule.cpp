/**
 * @file
 * @brief Remote Controller Source file
 *
 * RC Module
 *  - Finds the maximum and minimum values for each stick
 *  - Then saves these values inside of the EEPROM for 
 */


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "RcModule.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
unsigned int threshold_Low = 888,  // Thresholds to adjust for the RC remote controller - if under this, we do not read the signal
             threshold_On = 1500,  // Thresholds to adjust for the RC remote controller - if under it on channel 5, the remote is off
             microSec = 9760,  // Delay to wait between 2 data computation:
             RC_TimeOut = 25000,  // Time to read data on the receiver channel table PPM
             throttle = THROTTLE_MIN,  // Initial value in the case of corrupted data emission at the beginning:
             *p_throttle = &throttle,  // associated pointer
             rudder = (SIDEWAYS_MIN + SIDEWAYS_MAX)/2,  // Initial value in the case of corrupted data emission at the beginning:
             *p_rudder = &rudder;  // associated pointer

// Receiver Channel Table PPM value Structure
RemoteStruct Remote;

// Counter for Reset:
int countRestart = 0,
    *p_countRestart = &countRestart;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                    RC SETUP                      =
//  =                                                  =
//  ====================================================
void RcModuleSetup() {
  //Log(1, F("RcModuleSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  // Safety:
#ifndef RC_PIN_1
  // Generation of a compiler error:
  #error "RC_PIN_1" NOT DECLARED! see file "Wiring.h"
#endif

#ifndef RC_PIN_2
  // Generation of a compiler error:
  #error "RC_PIN_2" NOT DECLARED! see file "Wiring.h"
#endif

#ifndef RC_PIN_3
  // Generation of a compiler error:
  #error "RC_PIN_3" NOT DECLARED! see file "Wiring.h"
#endif
  pinMode(RC_PIN_1, INPUT);
  pinMode(RC_PIN_2, INPUT);
  pinMode(RC_PIN_3, INPUT);

  // Initialize Channel Table PPM values:
  Remote.status = 0;
  Remote.throttle = 0;
  Remote.rudder = 0;
}


//  ====================================================
//  =                                                  =
//  =                     RC LOOP                      =
//  =                                                  =
//  ====================================================
bool RcModuleLoop(unsigned int *p_throttle, unsigned int *p_rudder, bool lastVal) {
  Log(1, F("RcModuleLoop()"), F(""));

  String Thr = F("Throttle: "),
         Rud = F("Rudder:   ");
  
  // On/Off signal:
  ////////////////
  Remote.status = pulseIn(RC_PIN_3, HIGH, RC_TimeOut);
  // Delay to let time for value to be read:
  delayMicroseconds(microSec);


  //    ----------------------------------------------------
  //    -                                                  -
  //    -             Remote controller is Off             -
  //    -                                                  -
  //    ----------------------------------------------------
  if ((Remote.status > threshold_Low) && (Remote.status < threshold_On)) { // Remote controller is Off
    Log(0, F("Mode: "), F("Automatic"));
    //Message(F("\t"), F("Mode: "), F("Automatic"), 1);
    return (false);
  }


  //    ----------------------------------------------------
  //    -                                                  -
  //    -              Remote controller is On             -
  //    -                                                  -
  //    ----------------------------------------------------
  if (Remote.status > threshold_On) {
    Log(0, F("Mode: "), F("Manual"));
    //Message(F("\t\tMode:\tManual"), F(""), F(""), 1);
        
    // Throttle:
    ////////////
    // Reads value on the input pin:
    Remote.throttle = pulseIn(RC_PIN_2, HIGH, RC_TimeOut);  // (Pin, State, Time out)
    
    // Delay to let time for value to be read:
    delayMicroseconds(microSec);

    // Checks if data is corrupted or not:
    if (Remote.throttle > threshold_Low) {
      // If the "if" condition is not valid, the remote send false data, 
      // so we send again the previous command
      *p_throttle = Remote.throttle;
      
      // Generating Log:
      Log(2, F("Throttle Remote:"), String(*p_throttle));
    }
    else {
      // Generating Log:
      Log(2, F("Throttle Remote:"), F("    "));
    }
    
    // Generating the message:
    Thr += String(*p_throttle);
    

    // Rudder:
    //////////
    // Reads value on the input pin:
    Remote.rudder = pulseIn(RC_PIN_1, HIGH, RC_TimeOut);

    // Delay to let time for value to be read:
    delayMicroseconds(microSec);

    // Checks if data is corrupted or not:
    if (Remote.rudder > threshold_Low) {
      // If the "if" condition is not valid, the remote send false data, 
      // so we send again the previous command
      *p_rudder = Remote.rudder;

      // Generating Log:
      Log(2, F("Rudder Remote:"), String(*p_rudder));
    }
    else {
      // Generating Log:
      Log(2, F("Rudder Remote:"), F("    "));
    }
    
    // Generating the message:
    Rud += String(*p_rudder);

    //Sending Message:
    Message(F("Command:"), Thr, Rud, 1);


    //    ....................................................
    //    .                                                  .
    //    .                        Reset                     .
    //    .                                                  .
    //    ....................................................
    if ((Remote.rudder > (SIDEWAYS_MAX - ACCURACY)) && ((Remote.throttle < (THROTTLE_MIN + ACCURACY)) && (Remote.throttle != 0))) {
       // Countdown before Reset:
      *p_countRestart = countRestart + 1;
      Log(2, F("Countdown before Restart (restart when reaching 3):"), (String)countRestart);
    }
    else {  // Cancel countdown
     *p_countRestart = 0; 
    }

    if (countRestart >= 3) {  // Reset
      ResetArduino();
    }


    // End
    return (true);
  }
  

  //    ----------------------------------------------------
  //    -                                                  -
  //    -                   Corrupted Data                 -
  //    -                                                  -
  //    ----------------------------------------------------
  else { // Unknown data, so we send the last value of this function:
    return (lastVal);

    // Problem with RC remote controller or receiver
    //Warning(F("RcModuleLoop"), F("No data received"));
  }
}
