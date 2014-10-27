#ifndef TACHOMETER_H
#define TACHOMETER_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw messages
#include <messages/motor_status.h>
#include <messages/rpm.h>

//debug control
#define INCLUDE_TACH_PRINT 0

//*******************************************************************
//*                  TACHOMETER
//* Subscribe to motor status and convert the counts reported by
//* the Simple_plant into revolutions per minute.  Then publish
//* the Rpm_msg for feedback to the system.
//*******************************************************************

// external variables declared in the main sketch
extern Motor_status_msg motor_status_msg;
extern Rpm_msg rpm_msg;
extern Clearinghouse ch;

class Tachometer : public gw::Node {

private:
	//conversion factor to go from Time in ms to minutes for RPM
	static const double conv_time_to_min = 60000;
	
	//default set to 64.
	double cpr;				//counts per revolution

	Time last_update;
	Count last_ct;
	double last_rpm;
	
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Motor_status_msg> sub;
    Motor_status_msg local_status_msg;
	
	gw::Publisher<Rpm_msg> pub;
	Rpm_msg local_rpm_msg;
        
public:
	Tachometer()
		:Node("tach"),
		sub(&motor_status_msg, &ch, local_status_msg),
		pub(&rpm_msg, &ch, local_rpm_msg),
		cpr(64)
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}

	//Configure the counts per revolutions
	double counts_per_revolution() { return cpr; }
	void set_counts_per_revolution(double val) {cpr = val;}
    
	void run() {
		//update the local message
		sub.update();
		
		//perform the calculation
		Count new_ct = local_status_msg.ct;
		Count delta_ct = new_ct - last_ct;
		
		Time new_update = local_status_msg.timestamp;
		Time delta_time = new_update - last_update;
		
		double new_rpm = (double)delta_ct / (double)delta_time;
		new_rpm *= conv_time_to_min / cpr;
		
		//average with last value for meanpoint tracking
		new_rpm = (last_rpm + new_rpm) * .5;
		
		//update local msg
		local_rpm_msg.omega = new_rpm;
		local_rpm_msg.timestamp = new_update;
		
		//publish the message
		pub.publish();
		
		//set up for the next loop
		last_update = new_update;
		last_ct = new_ct;
		last_rpm = new_rpm;
	}
	
	#if INCLUDE_TACH_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.println(name());   
			Serial.print("\t");
			Serial.print(F("\tSubscribed to: "));
			sub.subscribed_where();
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
	    }
	#endif      
};

#endif
