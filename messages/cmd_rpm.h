#ifndef CMD_RPM_H
#define CMD_RPM_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         COMMAND RPM MESSAGE
//* The difference between this message and the similarly named
//* Rpm_msg is that this one is intended as the reference input to
//* a control loop, whereas rpm.h is intended as the feedback message
//* from a loop.
//************************************************************************

using namespace gw;

struct Cmd_rpm_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double omega;
	Time timestamp;
    
    //minimal constructor
    Cmd_rpm_msg() : Message("cmd_rpm"), 
		omega(0), timestamp(0) {}
	
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Cmd_rpm_msg* ptr = static_cast<Cmd_rpm_msg*>(msg);
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