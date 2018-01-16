#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"

ros::NodeHandle nh;
 
ros::Subscriber<geometry_msgs::Twist, Sailboat> sub("sailboat_cmd",&Sailboat::cmdCallback, Sailboat::Instance());
ros::Subscriber<std_msgs::String, Sailboat> sub2("sailboat_msg",&Sailboat::msgCallback, Sailboat::Instance());

unsigned long timerMillis = 0;

void setup() {
  
  Logger::MessagesSetup();
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  
  Sailboat::Instance()->init(nh);

  ControllerInterface* controllers[1];
  controllers[0] = new Controller(3);
  
  Sailboat::Instance()->setControllers(controllers,1);

  delay(10);
}

void loop() {
  if(millis() - timerMillis > 200){
    //Sailboat::Instance()->updateSensors();
    Sailboat::Instance()->updateTestSensors();
    Sailboat::Instance()->communicateData();
    Sailboat::Instance()->Control();
    timerMillis = millis();
  }

  nh.spinOnce();
  delay(20);
}
