#ifndef JUNCTION_H
#define JUNCTION_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/cmd_rpm.h>
#include <messages/rpm.h>
#include <messages/error.h>

//debug control
#define INCLUDE_JUNCTION_PRINT 1

//*******************************************************************
//*                        SUMMING JUNCTION
//* Simple block to replicate the usual function of a summing 
//* junction in a system diagram.  Subscribes to two messages and
//* publishes the differnce between them to a third message.
//*
//* I haven't designed any positive feedback systems within this
//* framework yet, so the summing junction is defaulted to subtract
//* the feedback signal from the ref signal.
//* 
//* I encourage anyone to add a configurable feature to this class
//* for positive or negative feedback...or even better extend it in
//* a way that allows control over multiple inputs.
//* 
//* Another great extension would be a templated version that isn't
//* limited to the double type for the error message....not hard, but
//* just another thing that needs to get done.
//*******************************************************************

// external variables declared in the main sketch
extern Cmd_rpm_msg cmd_rpm_msg;
extern Rpm_msg rpm_msg;
extern Error_msg error_msg;
extern Clearinghouse ch;

class Summing_junction : public gw::Node {

private:
	
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Cmd_rpm_msg> ref_sub;
    Cmd_rpm_msg local_ref_msg;

	gw::Subscriber<Rpm_msg> feedback_sub;
    Rpm_msg local_feedback_msg;
	
	gw::Publisher<Error_msg> pub;
	Error_msg local_error_msg;
        
public:
	Summing_junction()
		:Node("junction"),
		ref_sub(&cmd_rpm_msg, &ch, local_ref_msg),
		feedback_sub(&rpm_msg, &ch, local_feedback_msg),
		pub(&error_msg, &ch, local_error_msg)
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}
	    
	void run() {
		//update the local messages
		ref_sub.update();
		feedback_sub.update();
		
		//find the difference
		double epsilon = local_ref_msg.omega - local_feedback_msg.omega;
		local_error_msg.epsilon = epsilon;
		
		//update local Error msg
		local_error_msg.timestamp = millis();
		
		//publish the plant status message
		pub.publish();
	}
	
	#if INCLUDE_JUNCTION_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.println(name());   
			Serial.print("\t");
			Serial.print(F("\tSubscribed to: "));
			ref_sub.subscribed_where();
			Serial.print(F("\tSubscribed to: "));
			feedback_sub.subscribed_where();
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
	    }
	#endif      
};
#endif
