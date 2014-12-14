#ifndef BALANCE_PLANT_H
#define BALANCE_PLANT_H SEP_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/control_effort.h>
#include <messages/plant_status.h>

//gw components
#include <tools/quadrature.h>

//Supporting definitions
//MAX_EFFOR should be defined in the specific robot configuration file
//you are using to build balance plant around, but in case this is an
//example or the generic case I provide default definition here.  Just
//for clarification, the robot definition file should include all the 
//physical connections like motor connection pins, encoder connections,
//and the max effort that you expect your control solution to spit 
//out.  Then the balance plant will scale the max effort into the 
//range [0-255].  Same is true for the conversion factor from sensed
//current to Amps.
#ifndef MAX_EFFORT
	#define MAX_EFFORT 255
#endif
#ifndef CONV_I
	#define CONV_I 1
#endif

//debug control
#define INCLUDE_PLANT_PRINT 1

//*******************************************************************
//*                         BALANCE PLANT
//* Plant configured to drive a balancing robot with two
//* motors and encoders
//*******************************************************************

// external variables declared in the main sketch
extern Control_effort_msg control_effort_msg;
extern Plant_status_msg plant_status_msg;
extern Clearinghouse ch;

class Balance_plant : public gw::Node {

private:
     /*
        TODO Calibrate current sensing.
        Current sensing is based on Arduino Motor Control board's capability
        to deliver a maximum of two amps per channel.  I could test this for
        better accuracy.
     */
	
	//Vectors for supporting components
    Vector<Motor*> motors;
	Vector<Encoder*> encoders;
	
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Control_effort_msg> sub;
    Control_effort_msg local_effort_msg;
	
	gw::Publisher<Plant_status_msg> pub;
	Plant_status_msg local_plant_msg;
        
public:
	Balance_plant()
		:Node("balance_plant"),
		sub(&control_effort_msg, &ch, local_effort_msg),
		pub(&plant_status_msg, &ch, local_plant_msg)
	{
    	//motors.reserve(2);      //allocate memory for the motors vector
		//encoders.reserve(2);
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
		int effort = local_effort_msg.u;
		Direction::dir direction = (effort >= 0) ? Direction::fwd : Direction::bck;
		effort = abs(effort);
		effort = map(effort, 0,MAX_EFFORT,0,255);
		
		//drive the plant
		// char buf[50];
		// sprintf(buf, "direction is: %s",text(direction) );
		// Serial.println(buf);		
		for(int i = 0; i < motors.size(); ++i) {
			motors[i]->drive(direction, effort);
		}
		
		Serial.print("effort is: ");
		Serial.println(effort);
		
		
		//read the encoders 
		for(int i = 0; i < encoders.size(); ++i) {
			if(encoders[i]->pos() == Position::lt){
				local_plant_msg.lt_ct = encoders[i]->count();
			}
			if(encoders[i]->pos() == Position::rt){
				local_plant_msg.rt_ct = encoders[i]->count();
			}
		}		
		//read the motors
		for(int i = 0; i < motors.size(); ++i) {
			if(motors[i]->pos() == Position::lt){
				int val = motors[i]->current();
				local_plant_msg.lt_I = val * CONV_I;
			}
			if(motors[i]->pos() == Position::rt){
				int val = motors[i]->current();				
				local_plant_msg.rt_I = val * CONV_I;
			}
		}			
		//update local_plant_msg
		local_plant_msg.timestamp = millis();
		
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
			Serial.print(F("\tLocal msg is:"));
			local_effort_msg.print();
			Serial.print(F("\tLocal msg is:"));
			local_plant_msg.print();
	    }
	#endif
          
};

#endif
