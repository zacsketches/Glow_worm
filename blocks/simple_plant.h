#ifndef SIMPLE_PLANT_H
#define SIMPLE_PLANT_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/control_effort.h>
#include <messages/motor_status.h>

//gw components
#include <tools/quadrature.h>

//debug control
#define INCLUDE_PLANT_PRINT 0

//*******************************************************************
//*                         SIMPLE PLANT
//* Plant configured to drive a single motor an encoder
//*******************************************************************

// external variables declared in the main sketch
extern Control_effort_msg control_effort_msg;
extern Motor_status_msg motor_status_msg;
extern Clearinghouse ch;

class Simple_plant : public gw::Node {

private:
	
	// Max effort taken from model of controller in MATLAB simulation
	// to disturbances.
	static const int max_effort = 10000;	//rpm
	
	//Vectors for supporting components
    Vector<Motor*> motors;
	Vector<Encoder*> encoders;
	
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Control_effort_msg> sub;
    Control_effort_msg local_effort_msg;
	
	gw::Publisher<Motor_status_msg> pub;
	Motor_status_msg local_motor_msg;
        
public:
	Simple_plant()
		:Node("simple_plant"),
		sub(&control_effort_msg, &ch, local_effort_msg),
		pub(&motor_status_msg, &ch, local_motor_msg)
	{
    	motors.reserve(1);      //allocate memory for the motors vector
		encoders.reserve(1);
	}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {
		for(int i = 0; i < encoders.size(); ++i) {
			encoders[i]->begin();
		}
	}
	
    void attach(Motor* m) {
        motors.push_back(m);
    }
	
	void attach(Encoder* n){
		encoders.push_back(n);
	}
    
	void run() {
		//update the control effort
		sub.update();
		long effort = local_effort_msg.u;
		Direction::dir direction = (effort >= 0) ? Direction::fwd : Direction::bck;
		effort = abs(effort);
		effort = constrain(effort, 0, max_effort);
		effort = map(effort,0,max_effort,0,250);
		
		//drive the plant	
		for(int i = 0; i < motors.size(); ++i) {
			motors[i]->drive(direction, effort);
		}
		
		
		//read the encoders 
		for(int i = 0; i < encoders.size(); ++i) {
			local_motor_msg.ct = encoders[i]->count();
		}		
		//read the motors
		for(int i = 0; i < motors.size(); ++i) {
				int val = motors[i]->current();
				local_motor_msg.I = val;
		}			
		//update local_plant_msg
		local_motor_msg.timestamp = millis();
		
		//publish the plant status message
		pub.publish();
	}
	
	#if INCLUDE_PLANT_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.println(name());   
			Serial.print("\t");
			Serial.print(motors.size());
			Serial.println(F(" motors attached:"));
			for(int i = 0; i < motors.size(); ++i) {
				Serial.print(F("\t\t"));
				motors[i]->print();
			}
			Serial.print("\t");
			Serial.print(encoders.size());
			Serial.println(F(" encoders attached:"));
			for(int i = 0; i < encoders.size(); ++i) {
				Serial.print(F("\t\t"));
				encoders[i]->print();
			}
			Serial.print(F("\tSubscribed to: "));
			sub.subscribed_where();
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
	    }
	#endif      
};
#endif
