#include <RCModule.h>
#include <Arduino.h>
#include <Sailboat.h>
#include <Time.h>

void RC::testActivationStep(){
	switch(step){
		case 0:
		if(getValue(RC_1) == 1.0f){
			step++;
			timer = second();
			Logger::Instance()->Log(0,"Beginning", "RC Activation");
		}
		break;
		case 1:
		if(getValue(RC_1) == -1.0f){
			step++;
			timer = second();
		}
		break;
		case 2:
		if(getValue(RC_1) == 1.0f){
			step++;
			timer = second();
		}
		break;
		case 3:
		if(getValue(RC_1) < 0.25f && getValue(RC_1) > -0.25f){
			step++;
			timer = second();
		}
		break;
	}
}

void RC::updateMeasures(){
	if(second() - watchdog > 10 && controlling){
		Logger::Instance()->Log(0,"Changing Controller to : " + String(previousController), "RC Lost");
		controlling = false;
		Sailboat::Instance()->setController(previousController);
	}
	
	if(second() - timer > 10){
		if(step != 0){
			Logger::Instance()->Log(0,"Stopping - too late", "RC Activation");
			step = 0;
		}
	}
	
	if(Sailboat::Instance()->actualControllerIndex() == RC_CONTROLLER){
		controlling = true;
		step = 0;
	}else{
		controlling = false;
		previousController = Sailboat::Instance()->actualControllerIndex();
	}
	
	if(!controlling){
		testActivationStep();
		if(step > 4){
			controlling = true;
			watchdog = second();
			Sailboat::Instance()->setController(RC_CONTROLLER);
		}
	}
}

void RC::interruptCH(uint8_t channel, uint8_t pin){
	watchdog = second();
	if (digitalRead(pin) == HIGH) {
		rc_start[channel] = micros();
	} else {
		uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
		rc_values[channel] = rc_compare;
	}
}