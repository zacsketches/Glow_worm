/*
 * A series of LEDs driven off the PCF8575 I2C expander.
 * Demonstrates how to build a Glow Worm subscriber that controls hardware
 * via I2C. 
*/

#include <Wire.h>

#include <Vector.h>
#include <clearinghouse.h>
#include <messages/sensor_state.h>
#include <Light_plant.h>
#include <memory.h>

gw::Clearinghouse ch;

/*------------Set the message count for the system-----------------------*/
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

Sensor_state_msg sensor_state_msg;

Light_plant plant(1);

gw::Led led1("far_lt",  Port::p0);
gw::Led led2("near_lt", Port::p1);
gw::Led led3("mid",     Port::p2);
gw::Led led4("near_rt", Port::p3);
gw::Led led5("far_rt",  Port::p4);

void setup() {
  Serial.begin(57600);
  Serial.println();
  
  //Every gw::Node must call the begin() method.
  //Then attach the led to the plant
  plant.begin();
  plant.attach(&led1);
  plant.attach(&led2);
  plant.attach(&led3);
  plant.attach(&led4);
  plant.attach(&led5);
  
  //Manually register the sensor state message with the clearinghouse
  ch.register_msg(&sensor_state_msg);

  //set one value  
  sensor_state_msg.far_rt = LOB::obstructed;
  
  //Inspect the values of the Node and Message
  Serial.print("The sensor message is: \n\t");
  sensor_state_msg.print();
  Serial.print("The plant message is: \n");
  plant.print();
  
  plant.drive();
  
  //Inspect the amount of free memory
  Serial.print(F("Free RAM after the build: "));
  Serial.println(memory_free());
}

void loop() {
  // find a random number 
  long rnd1 = random(0,5);
  
  long rnd2 = random(0,2);
  
  LOB::lob tmp_lob = (rnd2 == 0) ? LOB::clear : LOB::obstructed;
  
  
  //turn the light on for that random number
  switch(rnd1) {
    case 0:
      sensor_state_msg.far_rt = tmp_lob;
      break;
    case 1:
      sensor_state_msg.near_rt = tmp_lob;
      break;
    case 2:
      sensor_state_msg.mid = tmp_lob;
      break;
    case 3:
      sensor_state_msg.near_lt = tmp_lob;
      break;
    case 4:
      sensor_state_msg.far_lt = tmp_lob;
      break;      
  }
  
  plant.drive();
  
  sensor_state_msg.print();
  
  delay(100);
}
