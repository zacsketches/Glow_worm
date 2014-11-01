#ifndef BUMPER_H
#define BUMPER_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         BUMPER MESSAGE
//************************************************************************

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
		Serial.print(F("{id: "));
		Serial.print(id());
        Serial.print(F(", name: "));
		Serial.print(name());
		Serial.print(F(", val: "));
		Serial.print(text(pressed));
        Serial.println(F("}"));
	}
	#endif
};

struct Two_bumper_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	Bump_state::bump_state pressed_lt;
	Bump_state::bump_state pressed_rt;
	
	const char* bumped;
	const char* clear;
    
    //minimal constructor
    Two_bumper_msg() : Message("2_bumper"), 
		pressed_lt(Bump_state::clear),
		pressed_rt(Bump_state::clear),
		bumped("bumped"), clear("clear")
		{}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Two_bumper_msg* ptr = static_cast<Two_bumper_msg*>(msg);
		pressed_lt = ptr->pressed_lt;
		pressed_rt = ptr->pressed_rt;
	}
	
	const char* text(Bump_state::bump_state val) {
		return (val == Bump_state::pressed ) ? bumped : clear;
	}
	
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("{id:"));
		Serial.print(id());
		Serial.print(F(", name:"));
		Serial.print(name());
		Serial.print(F(", lt:"));
		Serial.print(text(pressed_lt));
		Serial.print(F(", rt:"));
		Serial.print(text(pressed_rt));
        Serial.println(F("}"));
	}
	#endif
		
};




#endif