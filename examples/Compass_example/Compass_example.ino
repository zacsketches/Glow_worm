/*
 * Glow Worm robotics framework for Arduino.  Easy publish and
 * subscribe architecture for 8 bit microcontrollers.  
 *
 * Copyright 2014, Zac Staples <zacsketches @ github.com>
 * 
 * Demonstrates a Compass object publishing to the Heading_msg.
 *
 * Also uses my Filter class to implement a moving average low pass
 * filter to smooth the compass data a little.
 *
 * Requires an HMC5883 magnetic compass sensor connected to I2C in
 * order to use the existing libraries.  Other compasses could publish
 * in a similar manner, but the compass.cpp file would need to be
 * configured to the sensors specific registers
 *
*/

#include <Wire.h>
#include <Filter.h>
#include <Vector.h>
#include <Pair.h>
#include <clearinghouse.h>
#include <compass.h>
#include <messages/heading.h>

gw::Clearinghouse ch;

/*------------Set the message count for the system-----------------------*/
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

Heading_msg h1;

//const int compass_address = 0x1e;    //I2C 7 bit address for HMC5883
//Compass compass(compass_address, 5); //i2c address, moving avg filter length

Compass compass; //use default address and filter length of 5


void setup() {
  Serial.begin(57600);
  Serial.println();
  
  compass.begin();
  compass.print();
  delay(2500);
  	
}

void loop() {
  compass.run();
  
  h1.print();
  
  delay(50);
}
