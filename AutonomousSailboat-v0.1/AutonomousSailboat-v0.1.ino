#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"
#include "ReturnHome.h"
#include "Standby.h"
#include "RudderSailControl.h"

ros::NodeHandle nh;
 
ros::Subscriber<geometry_msgs::Twist, Sailboat> sub("sailboat_cmd",&Sailboat::cmdCallback, Sailboat::Instance());
ros::Subscriber<std_msgs::String, Sailboat> sub2("sailboat_msg",&Sailboat::msgCallback, Sailboat::Instance());

unsigned long timerMillis = 0;

void setup() {
  Logger::Instance()->MessagesSetup();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  
  Sailboat::Instance()->init(nh);

  ControllerInterface* controllers[4];
  controllers[0] = new Standby();
  controllers[1] = new RudderSail();
  controllers[2] = new ReturnHome();
  controllers[3] = new Controller(3);
  
  Sailboat::Instance()->setControllers(controllers,4);
  Sailboat::Instance()->setController(0);

  delay(10);
}

void loop() {
  Sailboat::Instance()->updateSensors();
  //Sailboat::Instance()->updateTestSensors();
  Logger::Instance()->Update();
  if(millis() - timerMillis > 200){
    Sailboat::Instance()->communicateData();
    Sailboat::Instance()->Control();
    //timerMillis = millis();
  }

  nh.spinOnce();
  delay(1);
}
