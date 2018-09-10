#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"
#include "ReturnHome.h"
#include "Standby.h"
#include "HeaderV.h"
#include "RudderSailControl.h"
#include "SailCapControl.h"
#include "RudderControl.h"
#include "RCControl.h"

#define EI_NOTPORTB
#define EI_NOTPORTJ
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <avr/wdt.h>
#include <EEPROM.h>

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist, Sailboat> sub("sailboat_cmd", &Sailboat::cmdCallback, Sailboat::Instance());
ros::Subscriber<std_msgs::String, Sailboat> sub2("sailboat_msg", &Sailboat::msgCallback, Sailboat::Instance());

const char signature [] = "Sailboat";
char * p = (char *) malloc (sizeof (signature));

void setControllers() {
  ControllerInterface* controllers[NB_CONTROLLERS];
  controllers[STANDBY_CONTROLLER] = new Standby();
  controllers[RUDDERSAIL_CONTROLLER] = new RudderSail();
  controllers[RETURNHOME_CONTROLLER] = new ReturnHome();
  controllers[HEADER_CONTROLLER] = new Header();
  controllers[RC_CONTROLLER] = new RCControl();
  controllers[SAILCAP_CONTROLLER] = new SailCap();
  controllers[RUDDER_CONTROLLER] = new RudderControl();
  controllers[C_CONTROLLER] = new Controller(3);

  Sailboat::Instance()->setControllers(controllers);
  Sailboat::Instance()->setController(STANDBY_CONTROLLER);
}

void intCH1() {
  Sailboat::Instance()->getRC()->interruptCH(RC_1, RC_PIN_1);
}

void intCH2() {
  Sailboat::Instance()->getRC()->interruptCH(RC_2, RC_PIN_2);
}

void intCH3() {
  Sailboat::Instance()->getRC()->interruptCH(RC_3, RC_PIN_3);
}

void intCH4() {
  Sailboat::Instance()->getRC()->interruptCH(RC_4, RC_PIN_4);
}

void intCH5() {
  Sailboat::Instance()->getRC()->interruptCH(RC_5, RC_PIN_5);
}

void intCH6() {
  Sailboat::Instance()->getRC()->interruptCH(RC_6, RC_PIN_6);
}

#ifdef WIND_ANEMOMETER_PIN
void AnemometerReading() {
  Sailboat::Instance()->getWindSensor()->updateAnemometer();
}
#endif

void setRCInterrupts() {
  pinMode(RC_PIN_1, INPUT);
  pinMode(RC_PIN_2, INPUT);
  pinMode(RC_PIN_3, INPUT);
  pinMode(RC_PIN_4, INPUT);
  pinMode(RC_PIN_5, INPUT);
  pinMode(RC_PIN_6, INPUT);
  enableInterrupt(RC_PIN_1, intCH1, CHANGE);
  enableInterrupt(RC_PIN_2, intCH2, CHANGE);
  enableInterrupt(RC_PIN_3, intCH3, CHANGE);
  enableInterrupt(RC_PIN_4, intCH4, CHANGE);
  enableInterrupt(RC_PIN_5, intCH5, CHANGE);
  enableInterrupt(RC_PIN_6, intCH6, CHANGE);
}

ISR(WDT_vect)
{
  EEPROM.write(1000,120);
}

void watchdogSetup(void)
{
  cli(); // disable all interrupts
  wdt_reset(); // reset the WDT timer
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
  sei();
}

bool checkIfColdStart() {
  /*if (strcmp (p, signature) == 0) { // signature is in RAM this was reset
    return false;
  }
  else {  // signature not in RAM this was a power on
    // add the signature to be retained in memory during reset
    memcpy (p, signature, sizeof signature);  // copy signature into RAM
    return true;
  }*/
  return (EEPROM.read(1000) == 0);
}

void setup() {
  Logger::Instance()->MessagesSetup();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);

  Sailboat::Instance()->init(&nh);

  delay(100);
  
  setControllers();
  setRCInterrupts();

#ifdef WIND_ANEMOMETER_PIN
  attachInterrupt(digitalPinToInterrupt(WIND_ANEMOMETER_PIN), AnemometerReading, FALLING);
#endif

  delay(10);

  if(checkIfColdStart()){
    //Clearing EEPROM is too long, clearing only a part
    for (int i = 0 ; i < 512; ++i)
      EEPROM.write(i, 0);
    EEPROM.write(1000,0);
    Sailboat::Instance()->getGPS()->informCold();
  }
  else{
    Sailboat::Instance()->setController(RETURNHOME_CONTROLLER);
    EEPROM.write(1000, 0);
  }

  watchdogSetup();

  if (LOGGER)
    Logger::Instance()->Toast("Sailboat is", "Ready!!", 0);
  //Sailboat::Instance()->publishMsg(String("Sailboat is Ready! Version : ") + String(VERSION_ARDUINO));
}

void loop() {
  wdt_reset();

  Sailboat::Instance()->updateSensors();
  //Sailboat::Instance()->updateTestSensors();
  Logger::Instance()->Update();
  Sailboat::Instance()->communicateData();
  Sailboat::Instance()->Control();
  

  nh.spinOnce();
  delay(4);
}
