#ifndef RPM_H
#define RPM_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         RPM MESSAGE
//************************************************************************

using namespace gw;

struct Rpm_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double omega;
	Time timestamp;
    
    //minimal constructor
    Rpm_msg() : Message("rpm"), 
		omega(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Rpm_msg* ptr = static_cast<Rpm_msg*>(msg);
		omega = ptr->omega;
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
		Serial.print(F(",omega:"));
		Serial.print(omega);
        Serial.println(F("}"));
	}
	#endif
		
};

#endif