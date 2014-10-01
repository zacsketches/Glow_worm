/*
  todo: this compiles all components...now I need to 
    -implement the WB_controller .cpp file
    - figure out how to interact/control it from the 
    main sketch.
    - okay...worked most of the afternoon....and have the controller
    working for everything up through the bump reaction....now I need
    to tailor the danger close reaction and the whisker reaction.
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
#include <controls/control_vec.h>
#include <WB_controller.h>
#include <Rover_plant.h>

//Supporting libraries
#include <Servo.h>
#include <memory.h>

//Physical connections
const int lt_bumper_pin = 13;
const int rt_bumper_pin = 12;
const int servo_pin = 6;
const int ping_pin = 7;
const int servo_ctr = 88;
const int span_width = 130;
const int servo_angular_rate = 8;  //fix this
const int rt_motor_dir = 2;
const int rt_motor_spd = 3;
const int lt_motor_dir = 4;
const int lt_motor_spd = 5;

/*------------Required to initiate the Glow Worm Framework---------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Bumper_msg lt_bumper_msg;
Bumper_msg rt_bumper_msg;
Five_pt_scan_msg five_pt_scan_msg;
Cmd_velocity_msg cmd_velocity_msg;

/*------------Construct the system components----------------------------*/
Scanner_5pt scanner(servo_pin, ping_pin, servo_ctr, span_width, servo_angular_rate);
Multi_bumper lt_bumper(lt_bumper_pin, &lt_bumper_msg, &ch);
Multi_bumper rt_bumper(rt_bumper_pin, &rt_bumper_msg, &ch);
Rover_plant rover(2);
Motor* mtr_lt = new Motor("lt", 4, 5);  //(name, dir_pin, pwm_pin)
Motor* mtr_rt = new Motor("rt", 2, 3);
WB_controller controller;

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
  
  controller.begin();
  controller.print();
  controller.show_trips();
  controller.show_scan_data();
  
  //attach motors to the Rover_plant
  rover.attach(mtr_lt);
  rover.attach(mtr_rt);
  rover.print();

  //Inspect the amount of free memory
  Serial.print(F("\nFree RAM after the build: "));
  Serial.println(memory_free());
  
  delay(2000);

}

void loop() {
  scanner.run();
//  five_pt_scan_msg.print();  

  lt_bumper.run();
  rt_bumper.run();

/*------Inspect controller data------*/  
  controller.run();
//  controller.show_trips();
//  controller.show_scan_data(); 
//  controller.show_sensor_state();
//  controller.show_dc();
  controller.show_bs();
//  controller.show_active_control();

/*------Inspect global messages-----*/  
  cmd_velocity_msg.print();
//  five_pt_scan_msg.print();
//  lt_bumper_msg.print();
//  rt_bumper_msg.print();
  
  rover.drive();
}