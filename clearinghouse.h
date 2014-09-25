#ifndef CLEARINGHOUSE_H
#define CLEARINGHOUSE_H SEP_2014

#include <arduino.h>
#include <Vector.h>

#define DEBUG_DRIVE       0
#define DEBUG_MOTOR_STATE 0

/*
 * Clearinghouse is the heart of a publish and suscribe architecture for the Arduino
 * microcontroller.  This class allows other nodes to publish and/or subscribe to
 * data.
 *
*/



//************************************************************************
//*                         Global Namespace enums
//************************************************************************


namespace Direction {
    enum dir {fwd, back};
	
    inline const char* text(dir d) {
      const char* res[5] = {(d == fwd) ? "frwd" : "back"};
      return res[0];
    }	
}

namespace State {
    enum state{low, high};
}


//Namespace Glow Worm
namespace gw {
	
//************************************************************************
//*                         MESSAGE
//************************************************************************
struct Message {
	/*
	 * Message is the base class for all system messages.  I'm not
	 * sure whether to use an ID or a name to identify these 
	 * messages, but will settle on a strategy as I begin to figure
	 * out the rest of the interface.
	 *  
	*/   
	static int msg_count;
	int msg_id;

    const char* n;        //name
    
    Message(const char* name) :n(name) {
    	msg_count++;
		msg_id = msg_count;
    }
    const char* name() const {return n;}
	const int id() const {return msg_id;}
    
    virtual void print() {
		Serial.print(id());
		Serial.print("\t");
		Serial.println(name());
    };
	
	// Updates the message from data in the passed message
	virtual void update(Message* msg) = 0;    
};


//************************************************************************
//*                         NODE
//************************************************************************
class Node {
/*
	TODO think about public/private with Node
*/
public:
	static int node_count;
	
	Node(const char* name): n(name){
		node_count++;
		node_id = node_count;
	}
    const char* name() const {return n;}
	const int id() const {return node_id;}
    
    virtual void print() {
		Serial.print(id());
		Serial.print("\t");
		Serial.print(name());
		Serial.println("\tcustom print note implemented for this node");
    };
	
	virtual void begin() = 0;
	
private:
	int node_id;
	const char* n; 
};

//************************************************************************
//*                         CLEARINGHOUSE
//************************************************************************
/*
	TODO should clearinghouse keep track of how many nodes are in the 
 *       systm and what they are?
*/
class Clearinghouse {

public:
	Clearinghouse();
	
	void register_msg(Message* msg);
	
	void list();
	
	void update(Message* msg);
	
	Message* get_ptr(const char* name);
		
private:
	Vector<Message*> store;

};

//************************************************************************
//*                         PUBLISHER
//************************************************************************
#define DEBUG_PUBLISH 0

template<class MSG>
class Publisher {
	/* 
	 * Must be constructed with a pointer to a message and
	 * a clearinghosue in the global scope of the sketch.  Often times
	 * this requires declaring the global variables 'extern' in the
	 * header file for the node.  For example, this line may be 
	 * necessary in the .h file that defines a node:
	 *     extern Simple_msg m1;
	 *     extern Clearinghouse ch;
	 * 
	 * In any node that is a publisher there must also be a local
	 * copy of the message.  When the publisher is created in the node
	 * then that local copy is passed in the Publisher's constructor
	 * to create a reference from the publisher to the local data
	 * calculated in the node.
	 * 
	 * Publisher logs the message with the clearinghouse and provides
	 * message handling methods to the node
    */
	
public:
	Publisher(Message* message, Clearinghouse* house, MSG& local_ref)
		: msg(message), ch(house), nodes_msg(local_ref) 
	{ ch->register_msg(msg); }

	/*
	 * When I publish the message I want the global copy of the
	 * message available via the pointer in the clearinghouse
	 * to be updated to reflect the latest info in local_msg
	*/  
	void publish(){
		//I need to call the update method on the message in
		//the clearinhouse and pass a pointer to the local message
		//publisher
		const char* local_name = nodes_msg.name();
		Message* store_msg_ptr = ch->get_ptr(local_name);
		store_msg_ptr->update(&nodes_msg); 
	}
	void publishing_where() const {
		msg->print();
	}

private:
	Message* msg;
	Clearinghouse* ch;
	MSG& nodes_msg;
};

//************************************************************************
//*                         SUBCRIBER
//************************************************************************
template<class MSG>
class Subscriber {
	/* 
	 * Must be constructed with a pointer to a message and
	 * a clearinghosue in the global scope of the sketch.  Often times
	 * this requires declaring the global variables 'extern' in the
	 * header file for the node.  For example, this line may be 
	 * necessary in the .h file that defines a node:
	 *     extern Simple_msg m1;
	 *     extern Clearinghouse ch;
	 * 
	 * In any node that is a Subscriber there must also be a local
	 * copy of the message.  When the Subscriber runs it's update()
	 * method it retrieves data from the global message and updates
	 * the local copy
	 * 
    */
	
public:
	Subscriber(Message* message, Clearinghouse* house, MSG& local_ref)
		: msg(message), ch(house), nodes_msg(local_ref) 
	{ }

	/*
	 * When I publish the message I want the global copy of the
	 * message available via the pointer in the clearinghouse
	 * to be updated to reflect the latest info in local_msg
	*/  
	void update(){
		// I need to call the update method on the local message
		// using the data in the clearinghouse message
		const char* local_name = nodes_msg.name();
		Message* store_msg_ptr = ch->get_ptr(local_name);
		nodes_msg.update(store_msg_ptr); 
	}
	
	void subscribed_where() const {
		msg->print();
	}

private:
	Message* msg;
	Clearinghouse* ch;
	MSG& nodes_msg;
};


//*******************************************************************
//*                         Motor_state
//*******************************************************************

struct Motor_state {
    Direction::dir d;
    int pwm;  
    
    Motor_state(Direction::dir direction = Direction::fwd, int speed = 0 )
        :d(direction), pwm(speed) {}

    void update(const Direction::dir direction, const int speed) {
        d = direction;
        pwm = speed;
    }

	void update(const Motor_state& ms){
		d = ms.d;
		pwm = ms.pwm;
	}
	#if DEBUG_MOTOR_STATE == 1
	    void print() {
	        char buf[20];
			sprintf(buf, "\tdir: %i\tspd: %i", d, pwm);
	        Serial.println(buf);
	    }
	#endif

};

//*******************************************************************
//*                         Motor
//*******************************************************************

class Motor {
    char* n;            //name
    int dp;            //dir_pin    
    int pp;            //pwm_pin
    Motor_state ms;
    
    int translate_dir(Direction::dir d) {
        /*
            TODO add a 'bool reverse' data member that flips the 
            return value below when set.
        */
        return (d == Direction::fwd) ? 0 : 1;
    }
    
public:
    Motor(char* name,
          int direction_pin,
          int pwm_pin,
          Direction::dir direction = Direction::fwd,
          int speed = 0)
          : n(name), dp(direction_pin), pp(pwm_pin), ms(direction, speed) {
              pinMode(dp, OUTPUT);
          }
    
   char* name() {return n;}    

   Motor_state state() const { return ms;}
   
   void set_state(Direction::dir direction, int speed) { 
       ms.update(direction, speed); 
   }
   
   void set_state(Motor_state new_state) { ms = new_state; }
   
   void drive() {
       int direction = translate_dir(ms.d);
       digitalWrite(dp, direction);
       analogWrite(pp, ms.pwm);
       #if DEBUG_DRIVE == 1
			Serial.print(F("@Drive debug"));
			char buf[50];
			sprintf(buf, "\t%s motor dir: %i\tspd: %i", n, ms.d, ms.pwm);
			Serial.println(buf);
	   #endif
   }
   
   void print() {
	   Serial.print(F("name: "));
	   Serial.print(name());
	   Serial.print(F("\tdir_pin: "));
	   Serial.print(dp);
	   Serial.print(F("\tpwm_pin: "));
	   Serial.println(pp);
   }
};
}

#endif

