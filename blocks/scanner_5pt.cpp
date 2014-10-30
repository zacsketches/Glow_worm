#include "scanner_5pt.h"

//*********************************************************************************
//                               SCAN CONSTRUCTOR
//*********************************************************************************
Scan::Scan(const int scan_points, const int span, const int center) 
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
        scan_pts.push_back(new gw::Scan_pt(head, 0));      
        if(i==0) ++multiple;
        if(i>0 && (i%2==0)) ++multiple;
        dir = -1 * dir; 
    }
}

const int Scan::heading_by_index(const int index) const {
    //return heading or -1 for index out of range
    int res = -1;

    if(index >= 0 && index < sz) {
        res = scan_pts[index]->heading();
    }

    return res;
}

const int Scan::range_by_index(const int index) const {
    //return heading or -1 for index out of range
    int res = -1;

    if(index >= 0 && index < sz) {
        res = scan_pts[index]->range();
    }

    return res;
}

bool Scan::update_by_heading(const int heading, const int data) {
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
Scanner_5pt::Scan_order::Scan_order(int test_points) {
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
Scanner_5pt::Scanner_5pt( const int servo_pin, 
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
void Scanner_5pt::begin() {
    servo.attach(sp);
    
    publish_message();
}

// RUN ... take data and store it.
void Scanner_5pt::run(){
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

//create a vector of the headings from outside the class
void Scanner_5pt::heading_vec(Vector<int>& dest) const {
    for(int i = 0; i < scan.size(); ++i) {
        int tmp = scan[i].heading();
        dest.push_back(tmp);
    }
}

#if INCLUDE_SCANNER_PRINT == 1 
void Scanner_5pt::print() {
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
	Serial.print(F("\tLocal msg is: "));
	local_msg.print();
}

#endif

//*******************************************************************
//*                 SCANNER HELPER FUNCTIONS
//*******************************************************************

void Scanner_5pt::publish_message() {
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
int Scanner_5pt::find_delay(const int target_angle) {
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
void Scanner_5pt::take_reading(const int heading) {
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
long Scanner_5pt::pulse() {
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
int Scanner_5pt::us_to_cm(const long microseconds)
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