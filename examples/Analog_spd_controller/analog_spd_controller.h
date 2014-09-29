#ifndef ANALOG_SPD_CONTROLLER_H
#define ANALOG SPD_CONTROLLER_H SEP_2014

#include <arduino.h>

#include <clearinghouse.h>
#include <cmd_velocity.h>

//*******************************************************************
//*                         ANALOG SPEED CONTROLLER
//* Read a pot on A0 and publish the cmd_velocity message accordingly
//*******************************************************************

extern Cmd_velocity_msg cmd_velocity_msg;
extern Clearinghouse ch;

class Analog_spd_controller : public Node {

private:
	int control_pin;

        bool test;

	Publisher<Cmd_velocity_msg> pub;
        Cmd_velocity_msg local_msg;   //local copy of the message
        
public:
	Analog_spd_controller(const int pin, bool test_mode=false)
		:Node("Anlg_spd_ctrl"),
		pub(&cmd_velocity_msg, &ch, local_msg),
		control_pin(pin)
	{
		//no pinMode setting required for Analog reads
	}
	
	// From Base class
    //    char* name()
	//    int id()

    //Required pure virtual from base
    void begin() {}
    
    void run() {
                static int val = 500;
                if(test) {
                  int multiple = random(0,2);
                  multiple = (multiple == 0 ) ? -1 : 1; 
                  val += multiple;
                  val = constrain(val, 0, 1023);
                } 
                else val = analogRead(control_pin);
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
