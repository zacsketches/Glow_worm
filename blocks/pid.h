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
	double last_epsilon;
	double Ti_inv;
	double Td;
	
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
	{
			if(Kp != 0) {
				Ti_inv = Ki/Kp;
				Td = Kd/Kp;
			} else {
				Ti_inv = 0;
				Td = 0;
			}
	}
	
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
   		double epsilon = local_error_msg.epsilon;

	    // Time management
	    Time delta_time = local_error_msg.timestamp - last_update;
		double dt = double(delta_time)/ timescale;
        
		//integrate
	    epsilon_integrated += epsilon * dt;
		
		//differentiate
		double de_dt = (last_epsilon - epsilon) / dt;
        
		//solve for adjustment
		//See http://en.wikipedia.org/wiki/PID_controller for a good
		//discussion of the standard form of the PID equation as used
		//in commercial controllers.
		/*
			u = Kp (epsilon + 1/Ti * integral[epsilong *dt] + Td * d/dt epsilon)
			
			Ti is integral time.  Ki = Kp / Ti
			Td is derivative time. Kd = Kp*Td
		*/

	    //double u = Kp * (epsilon + (Ti_inv*epsilon_integrated) + (Td*de_dt));
	    
		//Parallel form
		double u = Kp*epsilon + Ki*epsilon_integrated + Kd*de_dt;
        
		//update local msg
		local_effort_msg.u = (long)u;
		
		//publish the message
		pub.publish();
		
		//get ready for the next loop
		last_update = local_error_msg.timestamp;
		last_epsilon = epsilon;
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
