#ifndef PID_H
#define PID_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw messages
#include <messages/error.h>
#include <messages/control_effort.h>

//debug control
#define INCLUDE_PID_PRINT 1

#define TIMESCALE 1000.0            // use mills() for timing
//#define TIMESCALE 1000000.0       // use mircos() for timing

//*********************************************************
//*                  PID CONTROLLER
//* Will eventually implement a full PID controller, but 
//* today I just want to get the bones laid down with 
//* just proportional control
//*********************************************************

// external variables declared in the main sketch
extern Error_msg error_msg;
extern Control_effort_msg control_effort_msg;
extern Clearinghouse ch;

class Pid_controller : public gw::Node {

private:
	static const double timescale = TIMESCALE;
	
	double Kp;
	double Ki;
	double Kd;
	Time last_update;
	double epsilon_integrated;
	
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Error_msg> sub;
    Error_msg local_error_msg;
	
	gw::Publisher<Control_effort_msg> pub;
	Control_effort_msg local_effort_msg;
        
public:
	Pid_controller(double kp=1.0, double ki=0.0, double kd=0.0)
		:Node("pid"),
		sub(&error_msg, &ch, local_error_msg),
		pub(&control_effort_msg, &ch, local_effort_msg),
		Kp(kp), Ki(ki), Kd(kd)
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}
	
	void set_kp(float kp) {Kp = kp;}
	void set_ki(float ki) {Ki = ki;}
	void set_kd(float kd) {Kd = kd;}
	float kp() {return Kp;}
	float ki() {return Ki;}
	float kd() {return Kd;}

	void run() {
		//update the local message
		sub.update();
        
	    // Time management
	    Time delta_time = local_error_msg.timestamp - last_update;
		double dt = double(delta_time)/ timescale;
        
		//integrate
	    epsilon_integrated += local_error_msg.epsilon * dt;
        
		//solve for adjustment
	    double u = (Kp * local_error_msg.epsilon) + (Ki * epsilon_integrated);
        
		//update local msg
		local_effort_msg.u = (int)u;
		
		//publish the message
		pub.publish();
		
		//get ready for the next loop
		last_update = local_error_msg.timestamp;
	}
	
	#if INCLUDE_PID_PRINT == 1
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
