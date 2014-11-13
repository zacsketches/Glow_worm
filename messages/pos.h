#ifndef pos_h
#define pos_h

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         position MESSAGE
//* pos.h is intended as the feedback message
//* from a loop.
//************************************************************************

using namespace gw;

struct pos_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double distance_x;
	double distance_y;
	Time timestamp;
    
    //minimal constructor
    pos_msg() : Message("pos"), 
		distance_x(0), distance_y(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		pos_msg* ptr = static_cast<pos_msg*>(msg);
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