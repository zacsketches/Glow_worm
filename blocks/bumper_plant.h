#ifndef BUMPER_PLANT_H
#define BUMPER_PLANT_H OCT_2014

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/bump.h>

//gw components

//debug control
#define INCLUDE_BUMPER_PRINT 0

/* 
*  BUMPER_PLANT_H implements the simplest bumper system within my 
*  Glow Worm framework for Arduino robotics.  
* 
*  Bumpers are connected to a configured with INPUT_PULLUP.  
*  When grounded the pin indicates that the mobile robot has bumped into 
*  an object.
* 
*  In test mode the system has about a 1% chance of returning a bump.  Control
*  statistical likelihood of bump by constructing the gw::Bumper objects
*  in test mode.  See clearinghouse.h for details.
*
*  This bumper is implemented without interrupts so the run() method polls
*  the input pin and updates the published message if a bump is detected.
*/


//*******************************************************************
//*                         BUMPER_PLANT
//*******************************************************************

extern Two_bumper_msg bumper_msg;
extern Clearinghouse ch;

class Bumper_plant : public gw::Node {

private:
    //Vectors for supporting components
    Vector<gw::Bumper*> bumpers;
	
  	//Publisher/Subscribers and local copies
	gw::Publisher<Two_bumper_msg> pub;
    Two_bumper_msg local_msg;   //local copy of the plant message
        
public:
	Bumper_plant(int number_of_bumpers=2)
		:Node("bumper_plant"),
		pub(&bumper_msg, &ch, local_msg)
	{
    	bumpers.reserve(number_of_bumpers);      //allocate memory for the motors vector
		bumpers.push_back(NULL);
		bumpers.push_back(NULL);
	}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}	//required of any gw::Node...vestigial here
	
    void attach(gw::Bumper* b) {
        if(b->pos() == Position::lt) {
			bumpers[0] = b;
		}
		if(b->pos() == Position::rt) {
			bumpers[1] = b;
		}
    }
    
	void run() {
		local_msg.pressed_lt = bumpers[0]->status();	
		local_msg.pressed_rt = bumpers[1]->status();	
	
		pub.publish();
	}
	
	#if INCLUDE_BUMPER_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.print(name());   
			Serial.print("\t");
			Serial.print(bumpers.size());
			Serial.print(F(" bumpers attached: \n"));
			for(int i = 0; i < bumpers.size(); ++i) {
				Serial.print(F("\t"));
				bumpers[i]->print();
			}
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
	    }
	#endif          
};

#endif
