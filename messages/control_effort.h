#ifndef CONTROL_EFFORT_H
#define CONTROL_EFFORT_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         CONTROL_EFFORT MESSAGE
//************************************************************************

using namespace gw;

struct Control_effort_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	long u;
    
    //minimal constructor
    Control_effort_msg() : Message("cont_eff"), 
		u(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Control_effort_msg* ptr = static_cast<Control_effort_msg*>(msg);
		u = ptr->u;
	}
		
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("{id:"));
		Serial.print(id());
        Serial.print(F(",name:"));
		Serial.print(name());
		Serial.print(F(",U:"));
		Serial.print(u);
        Serial.println(F("}"));
	}
	#endif
		
};


#endif