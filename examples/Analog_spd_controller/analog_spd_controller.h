#ifndef ANALOG_SPD_CONTROLLER_H
#define ANALOG SPD_CONTROLLER_H SEP_2014

#include <arduino.h>

#include <clearinghouse.h>
#include <cmd_velocity.h>

//*******************************************************************
//*                         ANALOG SPEED CONTROLLER
//* Read a pot on A0 and publish the cmd_velocity message accordingly
//*******************************************************************

extern Cmd_velocity_msg cmd_velocity;
extern Clearinghouse ch;

class Analog_spd_controller : public Node {

private:
	int control_pin;
	Publisher<Cmd_velocity_msg> pub;
    Cmd_velocity_msg local_msg;   //local copy of the message
        
public:
	Analog_spd_controller(const int pin)
		:Node("Anlg_spd_ctrl"),
		pub(&cmd_velocity, &ch, local_msg),
		control_pin(pin)
	{
		//no pinMode setting required for Analog reads
	}
	
	// From Base class
    //    char* name()
	//    int id()
    
    void run() {
		int val = analogRead(control_pin);
		val = map(val, 0, 1023, 0, 255);
		local_msg.l_spd = val;
		local_msg.r_spd = val;
		pub.publish();
    }
    
    void print(){
		Serial.print(F("Node id: "));
		Serial.print(id());
		Serial.print("\t");
		Serial.print(name());   
		Serial.print("\t");
		Serial.print(F("Pot connected on A"));
		Serial.println(control_pin);
		Serial.print(F("\tPublishing to: "));
		pub.publishing_where();
		Serial.print(F("\tLocal msg is:"));
		local_msg.print();
    }
          
};

#endif