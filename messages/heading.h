#ifndef HEADING_MSG_H
#define HEADING_MSG_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>

//************************************************************************
//*                         HEADING MESSAGE
//************************************************************************
#define INCLUDE_PRINT 1

using namespace gw;

struct Heading_msg : public Message {
	// Data available from base class
	//      const char* name()
	//      int id()

	int heading;

	//Minimal constructor
	Heading_msg() : Message("heading"), heading(0) {}

	//assignment operator
	void update(Message* msg) {
		Heading_msg* ptr = static_cast<Heading_msg*>(msg);
		heading = ptr->heading;
	}

	#if INCLUDE_PRINT == 1
	//print
	void print() {
		Serial.print(F("id: "));
		Serial.print(id());
		Serial.print(F("\tname: "));
		Serial.print(name());
		Serial.print(F("\tval: "));
		Serial.println(heading);
	}
	#endif
};

#endif
