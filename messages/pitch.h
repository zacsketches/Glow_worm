#ifndef PITCH_H
#define PITCH_H OCT_2014


#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         PITCH MESSAGE
//************************************************************************

using namespace gw;

struct Pitch_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	int theta;
	uint32_t timestamp;
    
    //minimal constructor
    Pitch_msg() : Message("pitch"), 
		theta(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Pitch_msg* ptr = static_cast<Pitch_msg*>(msg);
		theta = ptr->theta;
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
		Serial.print(F(",theta:"));
		Serial.print(theta);
        Serial.println(F("}"));
	}
	#endif
		
};


#endif