#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"
#include "ReturnHome.h"
#include "Standby.h"
#include "HeaderV.h"
#include "RudderSailControl.h"
#include "RCControl.h"

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
  
  Sailboat::Instance()->setControllers(controllers,NB_CONTROLLERS);
  Sailboat::Instance()->setController(STANDBY_CONTROLLER);
}

void intCH1(){
  Sailboat::Instance()->getRC()->interruptCH(RC_1, RC_PIN_1);
}

void intCH3(){
  Sailboat::Instance()->getRC()->interruptCH(RC_3, RC_PIN_3);
}

void intCH5(){
  Sailboat::Instance()->getRC()->interruptCH(RC_5, RC_PIN_5);
}

void setRCInterrupts(){
  attachInterrupt(RC_PIN_1, intCH1, CHANGE);
  attachInterrupt(RC_PIN_3, intCH3, CHANGE);
  attachInterrupt(RC_PIN_5, intCH5, CHANGE);
}

void setup() {
  Logger::Instance()->MessagesSetup();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  
  Sailboat::Instance()->init(nh);
  
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
