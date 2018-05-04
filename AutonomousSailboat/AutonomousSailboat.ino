#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"
#include "ReturnHome.h"
#include "Standby.h"
#include "HeaderV.h"
#include "RudderSailControl.h"
#include "RCControl.h"

#define EI_NOTPORTJ 
#define EI_NOTPORTK
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>

ros::NodeHandle nh;
 
ros::Subscriber<geometry_msgs::Twist, Sailboat> sub("sailboat_cmd",&Sailboat::cmdCallback, Sailboat::Instance());
ros::Subscriber<std_msgs::String, Sailboat> sub2("sailboat_msg",&Sailboat::msgCallback, Sailboat::Instance());

void setControllers(){
  ControllerInterface* controllers[NB_CONTROLLERS];
  controllers[STANDBY_CONTROLLER] = new Standby();
  controllers[RUDDERSAIL_CONTROLLER] = new RudderSail();
  controllers[RETURNHOME_CONTROLLER] = new ReturnHome();
  controllers[HEADER_CONTROLLER] = new Header();
  controllers[RC_CONTROLLER] = new RCControl();
  controllers[C_CONTROLLER] = new Controller(3);
  
  Sailboat::Instance()->setControllers(controllers);
  Sailboat::Instance()->setController(STANDBY_CONTROLLER);
}

void intCH4(){
  Sailboat::Instance()->getRC()->interruptCH(RC_1, RC_PIN_4); //Trick because Hardware wrong
}

void intCH5(){
  Sailboat::Instance()->getRC()->interruptCH(RC_2, RC_PIN_5); //Trick because Hardware wrong
}

void intCH6(){
  Sailboat::Instance()->getRC()->interruptCH(RC_3, RC_PIN_6); //Trick because Hardware wrong
}

void setRCInterrupts(){
  //pinMode(RC_PIN_1, INPUT); //unused
  //pinMode(RC_PIN_2, INPUT); //unused
  //pinMode(RC_PIN_3, INPUT); //unused
  pinMode(RC_PIN_4, INPUT);
  pinMode(RC_PIN_5, INPUT);
  pinMode(RC_PIN_6, INPUT);
  //enableInterrupt(RC_PIN_1, intCH1, CHANGE); //unused
  //enableInterrupt(RC_PIN_2, intCH2, CHANGE); //unused
  //enableInterrupt(RC_PIN_3, intCH3, CHANGE); //unused
  enableInterrupt(RC_PIN_4, intCH4, CHANGE);
  enableInterrupt(RC_PIN_5, intCH5, CHANGE);
  enableInterrupt(RC_PIN_6, intCH6, CHANGE);
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
  
  delay(10);
}

void loop() {
  Sailboat::Instance()->updateSensors();
  //Sailboat::Instance()->updateTestSensors();
  Logger::Instance()->Update();
  Sailboat::Instance()->communicateData();
  Sailboat::Instance()->Control();

  nh.spinOnce();
  delay(1);
}
