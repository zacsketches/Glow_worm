#ifndef FIVE_PT_SCAN_H
#define FIVE_PT_SCAN_H JUN_2014


#include <arduino.h>
#include <Pair.h>
#include <clearinghouse.h>

//************************************************************************
//*                         5PT SCAN MESSAGE
//************************************************************************

#define INCLUDE_PRINT 1

using namespace gw;

struct Five_pt_scan_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()
	
//    typedef Pair<int, int> pair;	//<heading, range>
	
	Scan_pt p4;
	Scan_pt p2;
	Scan_pt p0;
	Scan_pt p1;
	Scan_pt p3;
	
            
    //minimal constructor
    Five_pt_scan_msg() : Message("5pt_scan") {}
    
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Five_pt_scan_msg* ptr = static_cast<Five_pt_scan_msg*>(msg);
	    p4 = ptr->p4;
	    p2 = ptr->p2;
	    p0 = ptr->p0;
	    p1 = ptr->p1;
	    p3 = ptr->p3;
	}	
    
	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("{id:"));
		Serial.print(id());
        Serial.print(F(",name:"));
		Serial.print(name());
		Serial.print(",data:[");
		Serial.print(text(p4));
		Serial.print(",");
		Serial.print(text(p2));
		Serial.print(",");
		Serial.print(text(p0));
		Serial.print(",");
		Serial.print(text(p1));
		Serial.print(",");
		Serial.print(text(p3));
		Serial.println("]}");
	}
	#endif

};

#endif
