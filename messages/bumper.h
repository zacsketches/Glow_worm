#ifndef BUMPER_H
#define BUMPER_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>


//************************************************************************
//*                         BUMPER MESSAGE
//************************************************************************

#define INCLUDE_PRINT 1

using namespace gw;

struct Bumper_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	bool pressed;
	const char* bumped;
	const char* clear;
    
    //minimal constructor
    Bumper_msg() : Message("bumper"), 
		pressed(false),
		bumped("bumped"),
		clear("clear") {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Bumper_msg* ptr = static_cast<Bumper_msg*>(msg);
		pressed = ptr->pressed;
	}
	
	const char* text(bool val) {
		return (val == true) ? bumped : clear;
	}
	
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("id: "));
		Serial.print(id());
		Serial.print(F("\tname: "));
		Serial.print(name());
		Serial.print(F("\tval: "));
		Serial.println(text(pressed));
	}
	#endif
		
};

#endif