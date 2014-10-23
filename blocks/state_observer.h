#ifndef STATE_OBSERVER_H
#define STATE_OBSERVER_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/plant_status.h>
#include <messages/pitch.h>
#include <messages/state_vec.h>

//gw components

//debug control
#define INCLUDE_OBSERVER_PRINT 1

//***********************************************************************
//                               STATE OBSERVER
// Collect data from the plant status message and the pitch message
// and formulate data to publish to the stote vector message
//***********************************************************************

// external variables declared in the main sketch
extern Plant_status_msg plant_status_msg;
extern Pitch_msg pitch_msg;
extern State_vec_msg state_vec_msg;
extern Clearinghouse ch;

class State_observer : public gw::Node {

private:

    //Publisher/Subscribers and local copies	
	gw::Subscriber<Plant_status_msg> plant_sub;
	Plant_status_msg local_plant_msg;  

    gw::Subscriber<Pitch_msg> pitch_sub;
    Pitch_msg local_pitch_msg;
    
    gw::Publisher<State_vec_msg> pub;
    State_vec_msg local_state_msg;
    
    //local data
    double theta;
    double theta_dot;
    double x;
    double x_dot;
    
    double last_theta;
    double last_theta_dot;
    double last_x;
    double last_x_dot;
    
    Time last_theta_time;
    Time theta_time;
    Time last_x_time;
    Time x_time;
    
public:
    State_observer() 
        : Node("state_observer"),
        plant_sub(&plant_status_msg, &ch, local_plant_msg),
        pitch_sub(&pitch_msg, &ch, local_pitch_msg),
        pub(&state_vec_msg, &ch, local_state_msg)
        {}
        
    // From Base class
    //    char* name()
	//    int id()
	
	void begin() {}
	
	void run() {
        //update the messages the node subscribes to
        plant_sub.update();
        pitch_sub.update();
        
        //compute theta
        theta = local_pitch_msg.theta;
        local_state_msg.theta = theta;
        
        //compute theta_dot
        theta_time = local_pitch_msg.timestamp;
        Time theta_dt = theta_time - last_theta_time;
        double delta_theta = last_theta - theta;
        theta_dot = delta_theta / theta_dt;
        local_state_msg.theta_dot = theta_dot; 
        
        //compute x
        // x is the average count of the two encoders
        /*
            TODO think about a better algorithm for x
            a pure average of the two encoders might not give the best results
            under circumstances where a wheel may slip or have undergone
            an intentional over-rotation to procue a twist.
        */
        x = (local_plant_msg.lt_ct + local_plant_msg.rt_ct) / 2;
        local_state_msg.x = x;
        
        //compute x_dot
        x_time = local_plant_msg.timestamp;
        Time x_dt = x_time - last_x_time;
        double delta_x = x - last_x;
        x_dot = delta_x / x_dt;
        local_state_msg.x_dot = x_dot;
        
        //shift the variables
        last_x = x;
        last_theta = theta;
        last_x_time = x_time;
        last_theta_time = theta_time;
        
        //publish the state vector
        pub.publish();
    }
    
    #if INCLUDE_PLANT_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.println(name());   
			Serial.print(F("\tSubscribed to: "));
			plant_sub.subscribed_where();
			Serial.print(F("\tSubscribed to: "));
			pitch_sub.subscribed_where();
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
		}
	#endif
    
};

#endif




