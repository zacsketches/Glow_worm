#ifndef CLEARINGHOUSE_H
#define CLEARINGHOUSE_H SEP_2014

#include <arduino.h>
#include <Vector.h>
#include <Pair.h>

#define DEBUG_DRIVE       0
#define DEBUG_MOTOR_STATE 0
#define DEBUG_SUBSCRIBER  0
#define DEBUG_PUBLISH     0

/*
	TODO add node_count() and message_count() functions
*/

/*
 * Clearinghouse is the heart of a publish and suscribe architecture for the Arduino
 * microcontroller.  This class allows other nodes to publish and/or subscribe to
 * data.
 ** The error
 *             wb_controller_example.cpp.o: In function `Subscriber':
 *             Arduino/.../Glow_worm/clearinghouse.h:273: undefined reference t
 * is usually caused by improper declaration of extern variables at the top of 
 * Glow Worm component header file.  For example the error above was caused by 
 * extern Cmd_velocity_msg cmd_velocity, when the correct name of the variable 
 * in the .ino file was 'cmd_velocity_msg'.
 *
 * The error
 * 		Arduino/../Glow_worm/clearinghouse.h:463: error: expected unqualified-id 
 *      before '<' token
 * is caused by not including Pair and Vector in sketch.
 *
 * Grateful thanks to Pololu for the inspiration and syntax behind the 
 * basic vector math functions here.  These are a modification to the outstanding
 * library they offer on their LG3 and LSM303 sensor carrier boards. Pololu 
 * license copied here:
		Copyright (c) 2014 Pololu Corporation.  For more information, see

		http://www.pololu.com/
		http://forum.pololu.com/

		Permission is hereby granted, free of charge, to any person
		obtaining a copy of this software and associated documentation
		files (the "Software"), to deal in the Software without
		restriction, including without limitation the rights to use,
		copy, modify, merge, publish, distribute, sublicense, and/or sell
		copies of the Software, and to permit persons to whom the
		Software is furnished to do so, subject to the following
		conditions:

		The above copyright notice and this permission notice shall be
		included in all copies or substantial portions of the Software.

		THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
		EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
		OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
		NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
		HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
		WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
		FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
		OTHER DEALINGS IN THE SOFTWARE.
 *
 * 
*/

//************************************************************************
//*                       GLOBAL TYPEDEFS AND ALIASES
//************************************************************************

typedef unsigned long Time;     //used in Time calculations
typedef long Count;				//used in motor encoding

//************************************************************************
//*                       GLOBAL CONSTANTS
//************************************************************************
namespace gw {

// I2C interface control variable. 
// Always check the status of this variable BEFORE calling Wire.begin().
// Code in the .begin() method of your Glow Worm blocks that require I2C
// should look something like this:
//		if(!gw::wire_begun) {
//			Wire.begin();
//			gw::wire_begun = true;
//		}

static bool wire_begun = false;

//Uncomment to keep track of ALL messages created.  Then static_cast the
//void* back into Message* in the main sketch.  Commented out because
//it takes up a lot of memory.
//static Vector<void*> full_msg_list;

}
//************************************************************************
//*                         Global Namespace enums
//************************************************************************


namespace Direction {
    //Used to define motor direction
	enum dir {fwd, bck};
	
    inline const char* text(dir d) {
      const char* res[5] = {(d == fwd) ? "frwd" : "back"};
      return res[0];
    }	
}

namespace Position {
	enum position{lt, rt, none};
}

namespace Led_state {
	//Used to define LED state
    enum led_state{off = 0, on = 1};
	
    inline const char* text(led_state s) {
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
	//Used to return a value from a bumper
	enum bump_state { clear = 0, pressed = 1};
}

namespace Danger_close_state {
    // Used in controller to set a bool value when the robot is 
	// within the danger close threshold and needs to react immediately
	enum dc {clear, danger_close};
	
	inline const char* const text(dc d) {
      const char* res[4] = {(d == clear) ? "clr" : "dcl"};
      return res[0];
    }
}


//Namespace Glow Worm
namespace gw {

//forward declaration
class Message;

//************************************************************************
//*                         CLEARINGHOUSE
//************************************************************************
class Clearinghouse {
public:
	Clearinghouse();
	
	void register_msg(Message* msg);
	
	void list();
	
	void update(Message* msg);
	
	Message* get_ptr(const char* name);
	
	Message* get_ptr(const int msg_id);
		
private:
	Vector<Message*> store;
};

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
		//Uncomment to keep track of ALL messages created.
		//gw::full_msg_list.push_back(this);

		msg_count++;
		msg_id = msg_count;	
	}
    const char* name() const {return n;}
	const int id() const {return msg_id;}
    
    virtual void print() {
		Serial.print(id());
		Serial.print(F("\t"));
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
		Serial.print(F("\t"));
		Serial.print(name());
		Serial.println(F("\tCustom print not implemented for this node"));
    };
	
	virtual void begin() = 0;
	
private:
	int node_id;
	const char* n; 
};

//************************************************************************
//*                         PUBLISHER
//************************************************************************

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
		const char* local_name = nodes_msg.name();
		Message* store_msg_ptr = ch->get_ptr(local_name);
		store_msg_ptr->update(&nodes_msg); 
	}
	void publish_by_id(){
		//I need to call the updated method on the message in the
		//ch with a pointer to the local message
		int id = msg->id();
//		Serial.print("the publishers id is: ");
//		Serial.println(id);
		Message* store_msg_ptr = ch->get_ptr(id);
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

		#if DEBUG_SUBSCRIBER == 1
				Serial.print(F("From Susciber: local_name is: "));
				Serial.println(local_name);
				Serial.print(F("From Susciber: ch msg node id is: "));
				Serial.println(store_msg_ptr->id());
		#endif
	}
	
	void update_by_id(){
		// I need to call the update method on the local message
		// using the data in the clearinghouse message
		int global_id = msg->id();
		Message* store_msg_ptr = ch->get_ptr(global_id);
		nodes_msg.update(store_msg_ptr); 

		#if DEBUG_SUBSCRIBER == 1
			Serial.print(F("the node is subscribed to id: "));
			Serial.println(global_id);
		#endif
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
//*                         Bumper
//* Represents a physical bumper on the robot that grounds a digital
//* pin when the bumper makes contact with an object.
//*******************************************************************

class Bumper {
private: 
	char* n;		//name
	int bp;			//bumper pin
	Position::position p;
	bool test;		//in test mode or not

	int one_in_x;
	
public:
	//default name
	Bumper(const int bumper_pin, Position::position pos, bool test_mode = false )
	    : n("bumper"), bp(bumper_pin), p(pos),
	    test(test_mode),
		one_in_x(100)   //default to 1% chance in test mode
		{ pinMode(bp, INPUT_PULLUP); }
	
	//named bumper
	Bumper(char* name, const int bumper_pin, Position::position pos, bool test_mode = false )
	    : n(name), bp(bumper_pin), p(pos),
	    test(test_mode),
		one_in_x(100)   //default to 1% chance in test mode
		{ pinMode(bp, INPUT_PULLUP); }
	
	Bump_state::bump_state status() {
		if(test) {
			int odds = random (0, one_in_x);
			return (odds == 1) ? Bump_state::pressed : Bump_state::clear; 
		} else {
			//recall that the pin grounds when pressed
			byte res = digitalRead(bp);
			return (res == HIGH) ? Bump_state::clear : Bump_state::pressed;
		}
	}	
	//Test mode probability
	int probability() const {return one_in_x;}
	void set_probability(const int k) {one_in_x = k;}
	
	//read position
	Position::position pos() const {return p;}
	//get name
	char* name() const { return n; }
	
	
	void print() {
	   Serial.print(F("{name:"));
	   Serial.print(name());
	   Serial.print(F(", pin:"));
	   Serial.print(bp);
	   Serial.println("}");
   }
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
private:
    const char* n;     //name
    int dp;            //dir_pin    
    int pp;            //pwm_pin
	int sp;			   //current sensor pin...must be an Analog input
	bool sense_enabled;
	int fwd_db;		  //forward deadband
	int rev_db;       //reverse deadband
	Position::position p;
	uint8_t forward_binary;
	uint8_t reverse_binary;
    Motor_state ms;
    
    int translate_dir(Direction::dir d) {
        return (d == Direction::fwd) ? forward_binary : reverse_binary;
    }
    
	int deadband_compensate(Direction::dir d, int raw_pwm) {
		int comp_pwm = 0;
		
		if( d == Direction::fwd ) {
			comp_pwm = map(raw_pwm, 0, 255, fwd_db, 255); 
		} else {
			comp_pwm = map(raw_pwm, 0, 255, abs(rev_db), 255);
		}
		
		return comp_pwm;
	}
	
public:
	// Constructor with no sense pin
    Motor(const char* name,
          int direction_pin,
          int pwm_pin,
		  Position::position pos = Position::none,
          Direction::dir direction = Direction::fwd,
          int speed = 0)
          : n(name), 
		  dp(direction_pin), 
		  pp(pwm_pin),
		  p(pos),
		  sp(0), 
		  sense_enabled(false),
		  fwd_db(0),
		  rev_db(0),
		  forward_binary(0),
		  reverse_binary(1),
		  ms(direction, speed) {
              pinMode(dp, OUTPUT);
          }
    
	// Constructor with sense pin
	Motor(const char* name,
	    int direction_pin,
	    int pwm_pin,
		int sense_pin,
	    Position::position pos = Position::none,
	    Direction::dir direction = Direction::fwd,
	    int speed = 0)
	    : n(name), 
		dp(direction_pin), 
		pp(pwm_pin), 
		sp(sense_pin), 
	    p(pos),
		sense_enabled(true),
		fwd_db(0),
		rev_db(0),		
	    forward_binary(0),
	    reverse_binary(1),
		ms(direction, speed) {
	        pinMode(dp, OUTPUT);
	    }  
    
	//Name
   const char* name() {return n;}    
	
   //Motor State
   Motor_state state() const { return ms;}
   void set_state(Direction::dir direction, int speed) { 
       ms.update(direction, speed); 
   }
   void set_state(Motor_state new_state) { ms = new_state; }
   
   //Current
   int current() const{
	   return (sense_enabled) ? analogRead(sp) : 0;
   }
   
   //Reverse
   //When a motor is physically installed and the leads cannot
   //be changed, this function will reverse the motors response
   //to Direction::[fwd|back]
   void reverse() {
	   forward_binary = !forward_binary;
	   reverse_binary = !reverse_binary;
   }
   
   //Deadband
   //All voltage regulated motors have a deadband where the
   //applied voltage is too small to overcome the static
   //friction of the motor and/or gears.  Setting the deadband
   //using the functions below sets the scale factors for the
   //linear scaling done to make sure small control commands
   //result in motor motion
   int forward_db() {return fwd_db;}
   int reverse_db() {return rev_db;}
   void set_deadband(int forward, int reverse){
		fwd_db=forward;
		rev_db=reverse;
   }
   
   //Pos
   Position::position pos() const { return p; }
   
   //Drive
   void drive() {
       int direction = translate_dir(ms.d);
       digitalWrite(dp, direction);
	   int pwm_sig = deadband_compensate(ms.d, ms.pwm);
       analogWrite(pp, pwm_sig);
       #if DEBUG_DRIVE == 1
			Serial.print(F("@Drive debug"));
			char buf[50];
			sprintf(buf, "\t%s motor dir: %i\tspd: %i", n, ms.d, pwm_sig);
			Serial.println(buf);
	   #endif
   }
   
   void drive(Direction::dir d, int raw_pwm) {
	   int direction = translate_dir(d);
	   int pwm_sig = deadband_compensate(d, raw_pwm);
 	   digitalWrite(dp, direction);
	   analogWrite(pp, pwm_sig);
	   #if DEBUG_DRIVE == 1
			Serial.print(F("@Drive debug"));
			char buf[50];
			sprintf(buf, "\t%s motor dir:%s\tspd:%i", n, text(d), pwm_sig);
			Serial.println(buf);
	   #endif
   }
   
   void print() {
	   Serial.print(F("name: "));
	   Serial.print(name());
	   Serial.print(F("\tdir_pin: "));
	   Serial.print(dp);
	   Serial.print(F("\tpwm_pin: "));
	   Serial.print(pp);
	   if(sense_enabled) {
		   Serial.print(F("\tsense_pin: "));
		   Serial.print(sp);
	   }
	   Serial.println();
   }
};

//*******************************************************************
//*                         LED
//* Led class allows me to declare an LED attached to the Arduino
//* as an object that can you can push_back into a vector.
//*******************************************************************

class Led {
    const char* n;      //name
    int p;              //pin the LED is attached to
    Led_state::led_state st;    //state of the LED either 'off' or 'on'
	bool i2c;
    
public:
    //Constructor for LED's attached directly to I/O pins
	Led(const char* name, int pin, Led_state::led_state s = Led_state::off) 
        :n(name), p(pin), st(s)
    {
        pinMode(p, OUTPUT);
		i2c = false;
    }
	
	//Constructor for LED's attached via an I2C expander
	Led(const char* name, Port::port pt, Led_state::led_state s = Led_state::off)
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
    
    Led_state::led_state state() { return st; }
    void set_state( Led_state::led_state cmd_state) { st = cmd_state; }

	void print() {
		if(i2c) 
			Serial.print(F("I2C "));
		Serial.print(F("Led"));
		if(i2c) {
			Serial.print(F(" connected on port:0x"));
			Serial.print(port(), HEX);
		}
		else {
			Serial.print(F(" connected on pin:"));
			Serial.print(pin());			
		}
		Serial.print(F(" state:"));
		Serial.print(text(state()));
		Serial.print(F(" name:"));
		Serial.println(name());
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
	
	void set_heading(const int h) {val_1 = h;}
    void set_range(const int r) {val_2 = r;}
	
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
	const char* end = NULL;
	char buf[10];
	sprintf(buf, "%03d:%03d", (int)sp.heading(), (int)sp.range() );
	Serial.print(buf);
	return end;
}

//Define an error point to check for return values of Scan points out of range
const Scan_pt ERROR_PT(999,999);

//************************************************************************
//*                         TRIP POINT
//************************************************************************
class Trip_pt : public Pair<int, int> {
private:
    byte f;  //the bit flag of the particular trip point that is
                // or'd to show all the tripped points in the sensor
public:
	Trip_pt(int heading = -1, int range = -1, byte flag=0x00) 
	    : Pair<int, int>(heading, range), f(flag) 
	{}
		
	const int heading() const {return first(); }
	const int range() const {return second(); }
    const byte flag() const { return f; }
	
    void set_heading(const int h) {val_1 = h;}
    void set_range(const int r) {val_2 = r;}
    void set_flag(const byte flag) {f = flag;}
};

inline const char* text(const Trip_pt& tp) {
    //print format of "000:000:0x00"
	char buf[14];
	sprintf(buf, "%03d:%03d:0x%02x", tp.first(), tp.second(), tp.flag() );
	return buf;
}

//************************************************************************
//*                         VECTOR MATH
//* vector data structure represents the notion of a vector in 
//* linear algebra.  Used in storing IMU data.  Vector functions
//* implement the basics of linear algebra
//************************************************************************

template <typename T> struct vector
{
  T x, y, z;
};

template <typename Ta, typename Tb, typename To> 
inline
void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);

static void vector_normalize(vector<float> *a);
static void vector_normalize(vector<int>* a);


template <typename Ta, typename Tb> 
inline
float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

template <typename Ta, typename Tb, typename Tr> 
inline 
void vector_add(const vector<Ta>* a, const vector<Tb>* b, vector<Tr>* result){
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;	  
}

template <typename Ta, typename Tb> 
inline 
void vector_add(vector<Ta>* a, const vector<Tb>* b){
    a->x = a->x + b->x;
    a->y = a->y + b->y;
    a->z = a->z + b->z;	  
}

template <typename T>
inline
void vector_scale(vector<T>& a, float scalar){
	a.x = a.x * scalar;
	a.y = a.y * scalar;
	a.z = a.z * scalar;
}

template <typename T>
inline
void vector_print(const vector<T>& a) 
{
	Serial.print(F("{x:"));
	Serial.print(a.x,4);
	Serial.print(F(", y:"));
	Serial.print(a.y,4);
	Serial.print(F(", z:"));
	Serial.print(a.z,4);
	Serial.println(F("}"));
}

template <typename Ta, typename Tb, typename To> 
inline
void vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

void vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

void vector_normalize(vector<int>* a) {
	vector<float> temp;
	temp.x = (float)a->x;
	temp.y = (float)a->y;
	temp.z = (float)a->z;
	
	Serial.print(F("before normalizing temp is: "));
	vector_print(temp);	
	vector_normalize(&temp);
	
	Serial.print(F("AFTER normalizing temp is: "));
	vector_print(temp);	
	
	a->x = (int)temp.x;
	a->y = (int)temp.y;
	a->z = (int)temp.z;
}

} //close namespace gw

#endif

