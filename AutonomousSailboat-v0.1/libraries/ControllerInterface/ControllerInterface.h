#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
			  
class ControllerInterface{
  public:
	ControllerInterface():activated(false){}
	virtual void init() = 0;
	virtual void Control(const geometry_msgs::Twist& cmd) = 0;
	
	virtual void updateBackground(){}
	
	bool isActivated(){return activated;}
	void setActivated(bool activ){activated = activ;}
  private:
	bool activated;
};

#endif
