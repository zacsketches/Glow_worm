#ifndef STATE_VEC_H
#define STATE_VEC_H OCT_2014


#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                        STATE VECTOR MESSAGE
//************************************************************************

using namespace gw;

struct State_vec_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	double theta;
	double theta_dot;
	double x;
	double x_dot;
    
    //minimal constructor
    State_vec_msg() : Message("state_vec"), 
		theta(0), theta_dot(0), x(0), x_dot(0) {}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		State_vec_msg* ptr = static_cast<State_vec_msg*>(msg);
		theta = ptr->theta;
		theta_dot = ptr->theta_dot;
		x = ptr->x;
		x_dot = ptr->x_dot;
	}
		
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print("{id:");
		Serial.print(id());
		Serial.print(",name:");
		Serial.print(name());
		Serial.print(",theta:");
		Serial.print(theta);
		Serial.print(",theta_dot:");
		Serial.print(theta_dot);
		Serial.print(",x:");
		Serial.print(x);
		Serial.print(",x_dot:");
		Serial.print(x_dot);
		Serial.println("}");
	}
	#endif
		
};


#endif