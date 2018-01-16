#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
			  
class ControllerInterface{
  public:
	virtual void init() = 0;
	virtual void Control(const geometry_msgs::Twist& cmd) = 0;
  private:
};

#endif
