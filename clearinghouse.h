#ifndef CLEARINGHOUSE_H
#define CLEARINGHOUSE_H SEP_2014

#include <arduino.h>
#include <Vector.h>
#include <Pair.h>

#define DEBUG_DRIVE       0
#define DEBUG_MOTOR_STATE 0
#define DEBUG_SUBSCRIBER  0

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
    //Used to define motor direction
	enum dir {fwd, back};
	
    inline const char* text(dir d) {
      const char* res[5] = {(d == fwd) ? "frwd" : "back"};
      return res[0];
    }	
}

namespace State {
	//Used to define LED state
    enum state{off = 0, on = 1};
	
    inline const char* text(state s) {
      const char* res[4] = {(s == off) ? "off" : "on "};
      return res[0];
    }
}

namespace LOB {		//Navy navigational reference to a line of bearing
	//Used to define whether a scan point is clear or obstructed
	enum lob{clear, obstructed};
	
    inline const char* const text(lob s) {
      const char* res[4] = {(s == clear) ? "clr" : "obs"};
      return res[0];
    }
}

namespace Port {
	//Used to define the ports on the I2C expander
	enum port{p0=0x01, 
	          p1=0x02, 
			  p2=0x04, 
			  p3=0x08, 
			  p4=0x10, 
			  p5=0x20, 
			  p6=0x40, 
			  p7=0x80, 
			  error};
}

namespace Bump_state {
    // Used in controller to set a bool value when the robot bumps into 
    // something and is maneuvering to get clear again.
	enum bs {clear, bumped};
	
	inline const char* const text(bs s) {
      const char* res[4] = {(s == clear) ? "clr" : "bum"};
      return res[0];
    }
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
#if DEBUG_SUBSCRIBER == 1
		Serial.print("From Susciber: local_name is: ");
		Serial.println(local_name);
#endif
		Message* store_msg_ptr = ch->get_ptr(local_name);
#if DEBUG_SUBSCRIBER == 1
		Serial.print("From Susciber: ch msg node id is: ");
		Serial.println(store_msg_ptr->id());
#endif
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

//*******************************************************************
//*                         LED
//* Led class allows me to declare an LED attached to the Arduino
//* as an actuator that can you can push_back into a vector.
//*******************************************************************

class Led {
    const char* n;      //name
    int p;              //pin the LED is attached to
    State::state st;    //state of the LED either 'off' or 'on'
	bool i2c;
    
public:
    //Constructor for LED's attached directly to I/O pins
	Led(const char* name, int pin, State::state s = State::off) 
        :n(name), p(pin), st(s)
    {
        pinMode(p, OUTPUT);
		i2c = false;
    }
	
	//Constructor for LED's attached via an I2C expander
	Led(const char* name, Port::port pt, State::state s = State::off)
		:n(name), p(pt), st(s)
	{
		i2c = true;
	}
    
	const char* name() {return n;}
    int pin() {return p;}

	Port::port port() {
		if(i2c) return (Port::port)p;
		else return Port::error;
	}
    
    State::state state() { return st; }
    void set_state( State::state cmd_state) { st = cmd_state; }

	void print() {
		if(i2c) 
			Serial.print(F("I2C "));
		Serial.print(F("Led"));
		if(i2c) {
			Serial.print(F(" connected on port: 0x"));
			Serial.print(port(), HEX);
		}
		else {
			Serial.print(F(" connected on pin: "));
			Serial.print(pin());			
		}
		Serial.print(F(" state: "));
		Serial.println(text(state()));
	}
};

//************************************************************************
//*                         SCAN POINT
//************************************************************************
class Scan_pt : public Pair<int, int> {
public:
	Scan_pt(int heading, int range) : Pair<int, int>(heading, range) {}
	
	Scan_pt() : Pair<int, int>(0,0) {}
	
	const int heading() const {return first(); }
	const int range() const {return second(); }
	
};

inline bool operator==(const Scan_pt& a, const Scan_pt& b ){
    bool res = false;
    if ( (a.heading() == b.heading()) && (a.range() == b. range()) ) {
        res = true;
    }
    return res;
}

inline bool operator!=(const Scan_pt& a, const Scan_pt& b ){
    return !(a==b);
}

inline const char* text(const Scan_pt& sp) {
	char buf[10];
	sprintf(buf, "%03d:%03d", sp.first(), sp.second() );
	return buf;
}

//Define an error point to check for return values of Scan points out of range
const Scan_pt ERROR_PT(999,999);

//************************************************************************
//*                         TRIP POINT
//************************************************************************
class Trip_pt : public Pair<int, int> {
public:
	Trip_pt(int heading, int range) : Pair<int, int>(heading, range) {}
	
	Trip_pt() : Pair<int, int>(-1,-1) {}
	
	const int heading() const {return first(); }
	const int range() const {return second(); }
	
};

inline const char* text(const Trip_pt& tp) {
	char buf[10];
	sprintf(buf, "%03d:%03d", tp.first(), tp.second() );
	return buf;
}

} //close namespace gw

#endif

