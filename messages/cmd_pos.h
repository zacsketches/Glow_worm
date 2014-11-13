#ifndef cmd_pos_h
#define cmd_pos_h

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                     COMMAND POSITION MESSAGE
//* The difference between this message and the similarly named
//* pos_msg is that this one is intended as the reference input to
//* a control loop, whereas pos.h is intended as the feedback message
//* from a loop.
//************************************************************************

using namespace gw;

struct Cmd_pos_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double distance_x;
	double distance_y;
	Time timestamp;
    
    //minimal constructor
    Cmd_pos_msg() : Message("cmd_pos"), 
		distance_x(0), distance_y(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Cmd_pos_msg* ptr = static_cast<Cmd_pos_msg*>(msg);
		distance_x = ptr->distance_x;
		distance_y = ptr->distance_y;
		timestamp = ptr->timestamp;
	}
		
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("{id:"));
		Serial.print(id());
        Serial.print(F(",name:"));
		Serial.print(name());
		Serial.print(F(",stamp:"));
		Serial.print(timestamp);
		Serial.print(F(",distance_x:"));
		Serial.print(distance_x);
		Serial.print(F(",distance_y:"));
		Serial.print(distance_y);
        Serial.println(F("}"));
	}
	#endif
};

#endif