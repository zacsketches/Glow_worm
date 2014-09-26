#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>

//************************************************************************
//*                         SENSOR STATE MESSAGE
//************************************************************************

#define INCLUDE_PRINT 1

using namespace gw;

struct Sensor_state_msg : public Message {

	// Data available from base class
	//      const char* name()
	//      int id()
	
    LOB::lob far_lt;
    LOB::lob near_lt;
    LOB::lob mid;
    LOB::lob near_rt;
    LOB::lob far_rt;
	
	//Constructor
	Sensor_state_msg() : Message("sensor_st"),
		far_rt(LOB::clear),
		near_rt(LOB::clear),
		mid(LOB::clear),
		far_lt(LOB::clear),
		near_lt(LOB::clear)
	{}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Sensor_state_msg* ptr = static_cast<Sensor_state_msg*>(msg);
	    far_lt = ptr->far_lt;
	    near_lt = ptr->near_lt;
	    mid = ptr->mid;
	    near_rt = ptr->near_rt;
	    far_rt = ptr->far_rt;
	}	
	
	#if INCLUDE_PRINT == 1
	//print
	void print() {
	    char buf[70];
	    sprintf(buf, "{id: %d, name: %s, state: %s, %s, %s, %s, %s}",
	        id(), name(), text(far_lt), text(near_lt), text(mid), text(near_rt), text(far_rt) );
	    Serial.println(buf);
	}
	#endif
};


#endif