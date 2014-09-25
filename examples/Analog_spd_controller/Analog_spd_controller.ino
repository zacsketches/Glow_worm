
/* 
 * Rover Motor Controller Conncted as follows:
 *   Rt dir on pin 2
 *   Rt PWM on pin 3
 *   Lt dir on pin 4 
 *   Lt PWM on pin 5
 */

#include <Vector.h>
#include <clearinghouse.h>
#include "cmd_velocity.h"
#include "Rover_plant.h"
#include "analog_spd_controller.h"

Clearinghouse ch;

/*------------Set the message count for the system-----------------------*/
int Message::msg_count = 0;
int Node::node_count = 0;

Cmd_velocity_msg cmd_velocity;

/*---------Construct Rover_plant and Motors------------------------*/
Rover_plant plant(2);                   //(2 motors)
Motor* mtr_lt = new Motor("lt", 4, 5);  //(name, dir_pin, pwm_pin)
Motor* mtr_rt = new Motor("rt", 2, 3);

Analog_spd_controller controller(0);    //pot on pin 0

/*---------Setup--------------------------------------------------*/
void setup() {
  Serial.begin(57600);
  Serial.println();
  ch.list();
  
  //attach motors to the Rover_plant
  plant.attach(mtr_lt);
  plant.attach(mtr_rt);
 
  //test the print function of the plant class
  plant.print(); 
  
  //inspect the total number of messages
  int total = Message::msg_count;
  Serial.print("Total messages: ");
  Serial.println(total);
  
  //inspect the speed controller
  controller.print();
  controller.run();
  controller.print();
  
}

void loop() {
  delay(10);  
  controller.run();
  plant.drive();
  
  //uncomment to see the messages from the plant
  // plant.print();
  
  //uncomment to see the messages from the controller
  // controller.print();
}