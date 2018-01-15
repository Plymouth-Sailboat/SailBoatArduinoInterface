#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H
			  
class ControllerInterface{
  public:
	virtual void init() = 0;
	virtual void Control() = 0;
  private:
};

#endif
