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
	
	Scan_pt far_lt;
	Scan_pt near_lt;
	Scan_pt mid;
	Scan_pt near_rt;
	Scan_pt far_rt;
	
            
    //minimal constructor
    Five_pt_scan_msg() : Message("5pt_scan") {}
    
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Five_pt_scan_msg* ptr = static_cast<Five_pt_scan_msg*>(msg);
	    far_lt = ptr->far_lt;
	    near_lt = ptr->near_lt;
	    mid = ptr->mid;
	    near_rt = ptr->near_rt;
	    far_rt = ptr->far_rt;
	}	
    
	#if INCLUDE_PRINT == 1
	//print
	void print() {
	    char buf[80];
	    sprintf(buf, "{id: %d, name: %s, (deg:cm) %s, ", id(), name(), text(far_lt));
	    strcat(buf, text(near_lt));
		strcat(buf, ", ");
	    strcat(buf, text(mid));
		strcat(buf, ", ");
	    strcat(buf, text(near_rt));
		strcat(buf, ", ");
	    strcat(buf, text(far_rt));
		strcat(buf, "}");
		
		Serial.println(buf);
	}
	#endif

};

#endif
