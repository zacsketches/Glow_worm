#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                      MOTOR_STATUS MESSAGE
//************************************************************************

using namespace gw;

struct motor_status_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	long  ct;
	int I;			//millivolts
	Time timestamp;
	    
    //minimal constructor
    Motor_status_msg() : Message("motor_status"), 
		ct(0), I(0.0), timestamp(0) {}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Motor_status_msg* ptr = static_cast<Motor_status_msg*>(msg);
		ct = ptr->ct;
		I = ptr->I;
		timestamp = ptr->timestamp;
	}
		
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		char buf[50];
		sprintf(buf, "{id:%d,name:%s,stamp:%d,ct:%d,I:%d}",
			id(), name(), timestamp, ct, I);
		Serial.println(buf);
	}
	#endif
		
};


#endif