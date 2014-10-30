#ifndef SCANNER_5PT_H
#define SCANNER_5PT_H SEP_2014

#include <arduino.h>  

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/five_pt_scan.h> 

//gw blocks and tools

//other libraries
#include <Servo.h>              //other libraries

//debug control
#define INCLUDE_SCANNER_PRINT 0
#define INCLUDE_SCAN_PRINT    0 
#define DEBUG_FIND_DELAY      0
#define DEBUG_RUN             0

/*
	TODO Build a simulation mode with much greater fidelity...the current
	simulation really sucks because it is too random.
*/

extern gw::Clearinghouse ch;
extern Five_pt_scan_msg five_pt_scan_msg;

/*
   Scanner extends the Glow Worm Node class to implement control over an ultrsonic 
   sensor mounted to a servo in order to scan multiple headings with a 
   single device.
   
   The extern variables above must be declared in the global scope of the main
   Arduino sketch.

   Servo angular rate is used to dermine how long to wait for the sensor to get into
   place before ordering a pulse.  If this value is too low then the sensor will
   pulse before the servo finishes moving the ultrasonic sensor.  The default value
   is computed from the speed of the Futaba 3004 servo that advertises an angular
   speed of .2sec/60deg.  or 200ms/60deg.  I default a little high to make sure the
   servo has plenty of time.
   
   Set bool test_mode to true in the constructor to generate random range data
   in the set [0, 360] and publish this data to the clearinghouse.
      
   The signature for creating a scanner is:
   Scanner(const int servo_pin, 
           const int ping_pin, 
           const int center = 90,
           const int span = 170,
           const int servo_angular_rate = 240/60+5,
           bool  test_mode = false); 
           
    The five point scanner publishes to the five_pt_scan message.

    You must call .begin() in the setup method of the Arduino sketch for all 
    Glow Worm nodes.

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
//* measurements.  Size equal to the number of scan points.
//*   
//* The span is constrained by the physical limits of the servo
//* to rotate the scanner.  Usually the span is constrained to a
//* quadrant ahead of the robot with the center point of the scan
//* parallel to the boresight of the robot centerline.
//*******************************************************************

class Scan 
{

    Vector<gw::Scan_pt*> scan_pts;
//    int* headings;                  //pointer to an array of headings
    int sz;                         //number of Scan_points in the scan
    int spn;                        //span of the scan in degrees

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
    
    // bool update_by_index(const int index, const int data);

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
};


//*******************************************************************
//*                         SCANNER CLASS
//* The Scanner class brings the data structures above together into 
//* a functional unit.  The Scanner, moves the servo to position the
//* sensor, initiates measurements from the ultrasonic sensor and
//* records the data into the Scan clas.
//*******************************************************************

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
    }; 

	Scan_order scan_order;
    
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
                           
	//Ultrasonic sensor connected on pin pp
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
	
	//A passed vector is filled data representing each scan point
    //This method allows other classes to get the heading info
    //used by the scanner....HOWEVER, the preferred method is to
    //get the headings out of the Five_pt_scan_msg that is published
    //
    //NOTE - this function is untested!
    void heading_vec(Vector<int>& dest) const;
	
	#if INCLUDE_SCANNER_PRINT == 1
        void print();
        void show_trips();
	#endif
  
};  

#endif
