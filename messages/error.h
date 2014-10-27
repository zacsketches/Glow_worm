#ifndef ERROR_H
#define ERROR_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         ERROR MESSAGE
//************************************************************************

using namespace gw;

struct Error_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double epsilon;
	Time timestamp;
    
    //minimal constructor
    Error_msg() : Message("error"), 
		epsilon(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Error_msg* ptr = static_cast<Error_msg*>(msg);
		epsilon = ptr->epsilon;
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
		Serial.print(F(",epsilon:"));
		Serial.print(epsilon);
        Serial.println(F("}"));
	}
	#endif		
};

#endif