/*
  todo: this compiles all components...now I need to 
    -implement the WB_controller .cpp file
    - figure out how to interact/control it from the 
    main sketch.
    - use the Vector<int>& function recently added to Scanner
    so that I can see the headings that are scanned and set up
    the trip points in the WB_controller constructor.
*/

//data structures
#include <Vector.h>
#include <Pair.h>

//Glow worm components
#include <clearinghouse.h>
#include <messages/cmd_velocity.h>
#include <messages/bumper.h>
#include <messages/five_pt_scan.h>
#include <Scanner_5pt.h>
#include <multi_bumper.h>
#include <Control_vec.h>
#include <WB_controller.h>

//Supporting libraries
#include <Servo.h>

//Physical connections
const int lt_bumper_pin = 10;
const int rt_bumper_pin = 11;
const int servo_pin = 6;
const int ping_pin = 9;
const int servo_ctr = 88;
const int span_width = 130;
const int servo_angular_rate = 240/60;

gw::Clearinghouse ch;

/*------------Set the message count for the system-----------------------*/
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Bumper_msg lt_bumper_msg;
Bumper_msg rt_bumper_msg;
Five_pt_scan_msg five_pt_scan_msg;
Cmd_velocity_msg cmd_velocity_msg;

/*------------Construct the system components----------------------------*/
Scanner_5pt scanner(servo_pin, ping_pin, servo_ctr, span_width, servo_angular_rate, true);
  Multi_bumper lt_bumper(lt_bumper_pin, &lt_bumper_msg, &ch, true);
  Multi_bumper rt_bumper(rt_bumper_pin, &rt_bumper_msg, &ch, true);

void setup() {
  Serial.begin(57600);
  Serial.println();
  
  ch.list();
  
  scanner.begin();
  scanner.print();
  
  lt_bumper.begin();
  lt_bumper.print();
  
  rt_bumper.begin();
  rt_bumper.print();
  
  delay(7500);

}

void loop() {
//  scanner.run();
//  five_pt_scan_msg.print();  
//  ch.list();
}
