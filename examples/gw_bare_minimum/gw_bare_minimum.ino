//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm components
#include <clearinghouse.h>

//other libraries (Servo, Wire, etc)
//#include <Wire.h>
//#include <Servo.h>

//Clearinhouse setup
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

void setup() {
  Serial.begin(57600);
  Serial.println();
  
//  Wire.begin();
//  gw::wire_begun = true;

}

void loop() {
  
}