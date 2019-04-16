#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
			  
class ControllerInterface{
  public:
	ControllerInterface():activated(false),timerControl(0),period(0){}
	ControllerInterface(int per):activated(false),timerControl(0),period(per){}
	virtual void init() = 0;
	void ControlTime(const geometry_msgs::Twist& cmd){if(millis()-timerControl > period){ Control(cmd); timerControl = millis();}}
	virtual void Control(const geometry_msgs::Twist& cmd) = 0;
	
	virtual void updateBackground(){}
	
	bool isActivated(){return activated;}
	void setActivated(bool activ){activated = activ;}
  protected:
	unsigned long period;
  private:
	bool activated;
	unsigned long timerControl;
};

#endif
