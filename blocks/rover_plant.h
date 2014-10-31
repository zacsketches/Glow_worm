#ifndef ROVER_PLANT_H
#define ROVER_PLANT_H SEP_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/cmd_velocity.h>

//gw components

//debug control
#define INCLUDE_ROVER_PRINT 0

//*******************************************************************
//*                         ROVER_PLANT
//* Plant configured to drive the the Dagu Rover5 mobile robot with
//* Dagu motor controller.
//*******************************************************************

extern Cmd_velocity_msg cmd_velocity_msg;
extern Clearinghouse ch;

class Rover_plant : public Node {

private:
    //Vectors for supporting components
    Vector<Motor*> motors;
	
  	//Publisher/Subscribers and local copies
	Subscriber<Cmd_velocity_msg> sub;
    Cmd_velocity_msg local_msg;   //local copy of the plant message
        
public:
	Rover_plant(int number_of_motors=2)
		:Node("rover_plant"),
		sub(&cmd_velocity_msg, &ch, local_msg)
	{
    	motors.reserve(number_of_motors);      //allocate memory for the motors vector
	}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}	//required of any gw::Node...vestigial here
	
    void attach(Motor* m) {
        motors.push_back(m);
    }
    
    void run() {
		sub.update();
        motors[0]->set_state(local_msg.l_dir, local_msg.l_spd);
        motors[1]->set_state(local_msg.r_dir, local_msg.r_spd);		
		
        for(int i = 0; i < motors.size(); ++i) {
            motors[i]->drive();
        }
    }
	
	#if INCLUDE_ROVER_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.print(name());   
			Serial.print("\t");
			Serial.print(motors.size());
			Serial.print(F(" motors attached: \n"));
			for(int i = 0; i < motors.size(); ++i) {
				Serial.print(F("\t"));
				motors[i]->print();
			}
			Serial.print(F("\tSubscribed to: "));
			sub.subscribed_where();
			Serial.print(F("\tLocal msg is:"));
			local_msg.print();
	    }
	#endif
          
};

#endif
