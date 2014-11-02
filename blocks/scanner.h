#ifndef SCANNER_H
#define SCANNER_H NOV_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/five_pt_scan.h> 

//gw components

//other 3rd party libraries
#include <Servo.h>

//debug control
#define INCLUDE_SCANNER_PRINT 0
#define INCLUDE_SCAN_PRINT    0 //turn on to use scan.ino in utilities
#define DEBUG_FIND_DELAY      0
#define DEBUG_RUN             0

/*
   Scanner extends the Glow Worm Node class to implement control over an ultrsonic 
   sensor mounted to a servo in order to scan multiple headings with a 
   single device.
   
   The extern variables for the clearinghouse and Five_pt_scan_msg
   must be declared in the global scope of the main Arduino sketch.

   Servo angular rate is used to dermine how long to wait for the sensor to get into
   place before ordering a pulse.  If this value is too low then the sensor will
   pulse before the servo finishes moving the ultrasonic sensor.  The default value
   is computed from the speed of the Futaba 3004 servo that advertises an angular
   speed of .2sec/60deg.  or 200ms/60deg.  I default a little high to make sure the
   servo has plenty of time to move.
   
   Set bool test_mode to true in the constructor to generate random range data
   in the set [0, 360] and publish this data to the clearinghouse.
      
   The signature for creating a scanner is:
   Scanner(const int servo_pin, 
           const int ping_pin, 
           const int center = 90,
           const int span = 170,
           const int servo_angular_rate = 240/60+5,
           bool  test_mode = false); 
           
    The five point scanner publishes to the Five_pt_scan_msg.

    You must call .begin() in the setup method of the Arduino sketch for all 
    Glow Worm nodes.  In scanner the .begin() method attaches the Servo.  The
	node will not function if begin is not called.

    The .run() method must be called each time the microcontroller loops in order to
    advance the scanner.  
    
    Measurements are in cm.
    
    Data storage is handled by the class with RAII principles.  No need for users to
    call delete or manage memory issues.
*/

//*******************************************************************
//*                         SCAN
//* The Scan class handles data for a scanner.  The primary container
//* is the Vector of *gw::Scan_pt.
//* 
//* The width of the span in degrees defines where the scanner can take 
//* measurements.  scan_pts.size() = the number of scan points.
//*   
//* The span is constrained by the physical limits of the servo
//* to rotate the scanner.  Usually the span is constrained to a
//* quadrant ahead of the robot with the center point of the scan
//* parallel to the boresight of the robot centerline.
//*******************************************************************

class Scan 
{

    Vector<gw::Scan_pt*> scan_pts;
    int sz;                         //number of Scan_points in the scan
    int spn;                        //span of the scan in degrees
	static const int clear_distance = 300;	//value to initialize the scan

public:  

    Scan(const int test_points, const int span=170, const int center=90);

    ~Scan() {}

  // NON-MODIFYING METHODS //
    int size() const { return sz; }
    int span() const { return spn; }
    
    const gw::Scan_pt operator[](const int index) const {
        gw::Scan_pt res = gw::ERROR_PT;
        
        if(index >= 0 && index < scan_pts.size() ){
            res = *scan_pts[index];
        }
        
        return res;
    }
    
    //caller must check -1 for out of bounds
    const int heading_by_index(const int index) const;    
    const int range_by_index  (const int index) const;    

  // MODIFYING METHODS //            
    //returns true on successful update
     bool update_by_heading(const int heading, const int data);  
    
    #if INCLUDE_SCAN_PRINT == 1
		void print_headings() {
			Serial.print(F("{Scan headings: "));
			for(int i = 0; i < sz; ++i) {
				int head = heading_by_index(i);
				Serial.print(head);
				if(i != (sz-1)) Serial.print(", ");
			}
			Serial.print(F("}\n"));
		}
		
		void print_ranges() {
			Serial.print(F("{Scan ranges: "));
			for(int i = 0; i < sz; ++i) {
				int range = range_by_index(i);
				char buf[4];
				sprintf(buf, "%3d", range);
				Serial.print(buf);
				if(i != (sz-1)) Serial.print(", ");
			}
			Serial.print(F("}\n"));
		}
	#endif
};


//*******************************************************************
//*                         SCANNER CLASS
//* The Scanner class brings the data structures above together into 
//* a functional unit.  The Scanner, moves the servo to position the
//* sensor, initiates measurements from the ultrasonic sensor and
//* records the data into the Scan class.
//*******************************************************************

extern gw::Clearinghouse ch;
extern Five_pt_scan_msg five_pt_scan_msg;

class Scanner_5pt : public gw::Node {
private:

	//Scan object handles data for the scanner
	Scan  scan;    

	//Scan_order object handles the order in which the scanner moves between Scan_points
	struct Scan_order{
		int* order;
		int pos;
		int sz;
		
		Scan_order(int test_points);   //test_points is the number of unique points in the scan
	                                      //the scanner will come back to the middle after every
	                                      //other measurement like the examples below for a 7 and 5 point scan
	                                      //where 0 is the middle
	                                      //      0 1 0 2 0 3 0 4 0 5 0 6
	                                      //      0 1 0 2 0 3 0 4
		~Scan_order() { delete[] order; } 

		Scan_order& operator++() {    //prefix ++
			++pos;
			if (pos == sz) pos = 0;
			return *this;
		}
		
        void restart() { pos = 0; }
		
        const int size() const {return sz;}
		const int current() const {return order[pos]; }
    } scan_order; 

	//Publisher and local copy of the message
	gw::Publisher<Five_pt_scan_msg> pub;
	Five_pt_scan_msg local_msg;    

	//Servo data
	int sp;                 //Arduino I/O pin where the servo signal is connected
	int ctr;                //Center in degrees where the servo points directly ahead
	int tp;                 //the number of test points used in the scan
    bool test;              //use an actual scanner or generate random data
	Servo servo;            //Servo object
	int sar;                //servo_angular_rate the servo turns at
	char servo_state;       // B0000 0001->servo ready 
    					    // B0000 0010->move ordered 
                            // B0000 0100->move complete
                            // B0000 1000->reading taken
                           
	//Ultrasonic Parallax PING))) sensor connected on pin pp
	int pp;
  
	//*** HELPER FUNCTIONS ***
    void publish_message();
	int find_delay(int target_angle);
	void take_reading(const int heading);       
	long pulse();
	int us_to_cm(const long duration); //microseconds_to_cm

public:
	//constructor
	Scanner_5pt( const int servo_pin, 
		const int ping_pin, 
		const int center = 90,
		const int span = 170,
		const int servo_angular_rate = (240/60+5),
        const bool test = false);
          
	// From Base class
	//    char* name()
	//    int id()
    
	void begin();
	void run();
	
	#if INCLUDE_SCANNER_PRINT == 1
        void print();
        void show_trips();
	#endif
  
};  

/* ------------------------------------IMPLEMENTATION----------------------------*/

/*
	Inline implementaion is necessary because I haven't found a way to link
	the .cpp file and still keep my different folders like blocks, messages...
*/

//*********************************************************************************
//                               SCAN CONSTRUCTOR
//*********************************************************************************
inline Scan::Scan(const int scan_points, const int span, const int center) 
    :sz(scan_points), spn(span)
{
    scan_pts.reserve(sz);

    //divide the span in angular sections and assign the test points
    int sections = sz - 1;
    int angular_seperation = spn / sections;

    //scan point zero is at the center of the scan with even points
    //to the left (port) and odd points to right (starboard)...can't
    //help it..Navy habits die hard
    int head = center;
    int dir = 1;
    int multiple = 0;
    for(int i = 0; i <sz; ++i) {
        head = center + (dir * multiple * angular_seperation); 
        scan_pts.push_back(new gw::Scan_pt(head, clear_distance));      
        if(i==0) ++multiple;
        if(i>0 && (i%2==0)) ++multiple;
        dir = -1 * dir; 
    }
}

inline const int Scan::heading_by_index(const int index) const {
    //return heading or -1 for index out of range
    int res = -1;

    if(index >= 0 && index < sz) {
        res = scan_pts[index]->heading();
    }

    return res;
}

inline const int Scan::range_by_index(const int index) const {
    //return heading or -1 for index out of range
    int res = -1;

    if(index >= 0 && index < sz) {
        res = scan_pts[index]->range();
    }

    return res;
}

inline bool Scan::update_by_heading(const int heading, const int data) {
    //return false on bad update
    //returns true on successful update
    bool success = false;
    for(int i = 0; i < sz; ++i) {
        //find index for given heading
        if (scan_pts[i]->first() == heading) {
            delete scan_pts[i];
            scan_pts[i] =new gw::Scan_pt(heading, data);
            success = true;
            break;
        }
    }
    return success;

}

//*******************************************************************
//*                         SCAN ORDER CONSTRUCTOR
//*******************************************************************
inline Scanner_5pt::Scan_order::Scan_order(int test_points) {
    pos = 0;
    sz = 2* (test_points -1);
    order = new int[sz];
    int mult = 1;
    for(int i = 0; i < sz; ++i) {
        if(i%2 == 0) order[i] = 0;    //always come back to the middle
        else {
            order[i] = mult;
            ++mult;
        }
    }     
}

//*******************************************************************
//*                     SCANNER CONSTRUCTOR
//* Note that 5 test points are hard coded at construction
//*******************************************************************
inline Scanner_5pt::Scanner_5pt( const int servo_pin, 
                 const int ping_pin, 
                 const int center,
                 const int span,
                 const int servo_angular_rate,
                 const bool test_data)
    : Node("Scanner_5pt"),
      sp(servo_pin), pp(ping_pin), ctr(center), tp(5), 
      scan(5, span, center), sar(servo_angular_rate), test(test_data),
      scan_order(5),
	  pub(&five_pt_scan_msg, &ch, local_msg)
{

    servo_state = 0x01;     // Ready
    //running the Servo::attach(int) function for the servo from 
    // the Scanner constructor yields undefined servo behavior.  
}

//*********************************************************************************
//                   SCANNER CLASS METHODS
//*********************************************************************************

// Begin must be called for all Glow Worm nodes in the sketch setup()
inline void Scanner_5pt::begin() {
    servo.attach(sp);
    
    publish_message();
}

// RUN ... take data and store it.
inline void Scanner_5pt::run(){
    static unsigned long command_time;	//time servo ordered to move
    static unsigned long ready_time;	//time servo will be ready
    static int target_heading = 90;
    
    //if servo is ready then order next scan
    if(servo_state & 0x01 == 0x01) {	    //B0001 tests servo_ready bit
        target_heading = scan.heading_by_index( scan_order.current() );
        command_time=millis();		        //record time move was ordered
        ready_time = command_time + find_delay(target_heading);
        servo.write(target_heading);		//order servo to move
        servo_state = 0x02;		           //set state to B0000 0010->move ordered
        #if DEBUG_RUN == 1
            Serial.println("Debug from run():");
            Serial.print("\ttarget_heading is: ");
            Serial.println(target_heading);
            Serial.print("\tcommand time is: ");
            Serial.println(command_time);
            Serial.print("\tready time is: ");
            Serial.println(ready_time);
            Serial.print("\tservo state is: 0x");
            Serial.println(servo_state, HEX);
        #endif          
    }
    
    //if servo move ordered and time has elapsed then move is complete
    if((servo_state == 0x02) && ( millis()>=ready_time) ) { //B0000 0010->move_ordered
        #if DEBUG_SER == 1
            Serial.println("Scanner run report: ");
            Serial.print("\tmove ordered and time expired.  servo_state is: ");
            Serial.println(servo_state, HEX);
        #endif
        servo_state = 0x04; 	//B0000 0100->move_complete
    }
    
    //if servo move complete, then take reading
    if(servo_state == 0x04) {	//B0000 0100->move_complete
        take_reading(target_heading);
        servo_state = 0x08;	    //B0000 1000->reading taken
        #if DEBUG_SER == 1
          Serial.println("Scanner run report: ");
          Serial.print("\tping complete and servo_state is: ");
          Serial.println(servo_state, HEX);
        #endif
        #if DEBUG_SER == 1
          char buf[50];
          sprintf(buf, "\theading:distance just measured %03d:%d", target_heading, 
              scan.data_by_heading(target_heading));
          Serial.println( buf );
        #endif
    }

    //if reading taken, then advance scan_order and publish the message
    if(servo_state == 0x08) {  //B0000 1000->reading taken
        ++scan_order;
        publish_message();
        servo_state = 0x01;    //B0000 0001->ready
    }
}


#if INCLUDE_SCANNER_PRINT == 1 
inline void Scanner_5pt::print() {
    char buf[75];

    //print config data to serial
    Serial.println(F("Five point scanner configuration:"));
    Serial.print(F("\tNode id is: "));
    Serial.println(id());
    if(test) Serial.println("\tIn test mode.");
    else Serial.println("\tIn run mode.");
    sprintf(buf, "\tServo connected on pin: %d", sp);
    Serial.println(buf);
    sprintf(buf, "\tPing sensor connected on pin: %d", pp);
    Serial.println(buf);
    sprintf(buf, "\tThe scan is %3d degrees centered at %3d with %d test points.", 
        scan.span(), ctr, scan.size());
    Serial.println(buf);
    Serial.println(F("\tThe readings vector contains the following headings:"));
    Serial.print("\t");
    for(int i = 0; i < scan.size(); ++i) {
        Serial.print("\t");
        Serial.print(scan.heading_by_index(i));
    }
    Serial.println();
    Serial.println(F("\tThe scan order is:"));
    Serial.print("\t");
    scan_order.restart();
    for(int i = 0; i <= scan_order.size(); ++i) {
        Serial.print("\t");
        Serial.print(scan.heading_by_index(scan_order.current()));
        ++scan_order;
    }
    scan_order.restart();
    Serial.println();
    Serial.print(F("\tPublishing to: "));
	pub.publishing_where();
}

#endif

//*******************************************************************
//*                 SCANNER HELPER FUNCTIONS
//*******************************************************************

inline void Scanner_5pt::publish_message() {
    //update the local copy from the data in the Scan
    if(scan[0] != gw::ERROR_PT)
        local_msg.p0 = scan[0];
    if(scan[1] != gw::ERROR_PT)
        local_msg.p1 = scan[1];
    if(scan[2] != gw::ERROR_PT)
        local_msg.p2 = scan[2];
    if(scan[3] != gw::ERROR_PT)
        local_msg.p3 = scan[3];
    if(scan[4] != gw::ERROR_PT)
        local_msg.p4 = scan[4];
    
    //then publish the message
    pub.publish();
}

// RETURN DELAY IN MILLISECONDS FOR THE SERVO TO MOVE BETWEEN TWO ANGLES
inline int Scanner_5pt::find_delay(const int target_angle) {
	int current_angle = servo.read();
	#if DEBUG_FIND_DELAY == 1
      Serial.println(F("Debug from find_delay():"));
	  Serial.print("\tcurrent_angle is: ");
	  Serial.println(current_angle);
	  Serial.print("\ttarget_angle is: ");
	  Serial.println(target_angle);
	#endif
	int theta = target_angle - current_angle;
	theta = abs(theta);
	#if DEBUG_FIND_DELAY == 1
	  Serial.print("\ttheta is: ");
	  Serial.println(theta);
	#endif
    #if DEBUG_FIND_DELAY == 1
      Serial.print("\tsar is: ");
      Serial.println(sar);
    #endif
	int delay = theta * sar;
    #if DEBUG_FIND_DELAY == 1
      Serial.print("\tdelay is: ");
      Serial.println(delay);
    #endif	
	return delay;	//in milliseconds
}

// TAKE A MEASURMENT AND UPDATE THE SCAN OBJECT
inline void Scanner_5pt::take_reading(const int heading) {
    //the servo is already in the right position when this method is
    //called so we just need to bang the sonar, get the measurement
    //and then save it to the scan
    long dur;
    if(!test) dur = pulse();
    else dur = random(0,20880);
    
    int cm = us_to_cm(dur);
    
    scan.update_by_heading(heading, cm);
}

// PULSE THE SENSOR AND RETURN THE DATA
inline long Scanner_5pt::pulse() {
    //standard pulse method for Parallax PING))) sensor
    pinMode(pp, OUTPUT);
    digitalWrite(pp, LOW);
    delayMicroseconds(2);
    digitalWrite(pp, HIGH);
    delayMicroseconds(5);
    digitalWrite(pp, LOW);

    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pp, INPUT);
    long duration = pulseIn(pp, HIGH);

    return duration;
}

// CONVERT ROUND TRIP MICROSECONDS TO CM
inline int Scanner_5pt::us_to_cm(const long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  /*
    TODO I could calibrate this and take temp and pressure into account for a
    more accurate sound speed calculation.
  */
  return microseconds / 29 / 2;
}

#endif
