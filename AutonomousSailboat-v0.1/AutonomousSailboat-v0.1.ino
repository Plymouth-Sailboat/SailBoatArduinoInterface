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
  ControllerInterface* controllers[6];
  controllers[0] = new Standby();
  controllers[1] = new RudderSail();
  controllers[2] = new ReturnHome();
  controllers[3] = new Header();
  controllers[4] = new RCControl();
  controllers[5] = new Controller(3);
  
  Sailboat::Instance()->setControllers(controllers,6);
  Sailboat::Instance()->setController(0);
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
