#ifndef PLANT_STATUS_H
#define PLANT_STATUS_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                      PLANT_STATUS MESSAGE
//************************************************************************

using namespace gw;

struct Plant_status_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
	long  lt_ct;
	float lt_I;			//millivolts
	long  rt_ct;
	float rt_I;
	Time timestamp;
	    
    //minimal constructor
    Plant_status_msg() : Message("plant_status"), 
		lt_ct(0), lt_I(0.0), rt_ct(0), rt_I(0.0), timestamp(0) {}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Plant_status_msg* ptr = static_cast<Plant_status_msg*>(msg);
		lt_ct = ptr->lt_ct;
		lt_I = ptr->lt_I;
		rt_ct = ptr->rt_ct;
		rt_I = ptr->rt_I;
		timestamp = ptr->timestamp;
	}
		
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		char buf[85];
		sprintf(buf, "{id:%d,name:%s,stamp:%d,lt_ct:%d,lt_I:%f,rt_ct:%d,rt_I:%f}",
			id(), name(), timestamp, lt_ct, lt_I, rt_ct, rt_I);
		Serial.println(buf);
	}
	#endif
		
};


#endif