#include <Pair.h>
#include <Vector.h>
#include <clearinghouse.h>
#include <messages/bumper.h>
#include <Simple_bumper.h>

gw::Clearinghouse ch;

const int bumper_pin = 8;  //bumper or momentary switch connected on this pin

/*------------Set the message count for the system-----------------------*/
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

Bumper_msg bumper_msg;

Simple_bumper bumper(bumper_pin);

void setup() {
  Serial.begin(57600);
  Serial.println();
  
  //Must call begin on any Node!!!
  bumper.begin();
  
  //Inspect the values of the Node and Message
  bumper_msg.print();
  bumper.print();

  delay(2500);
}

void loop() {
  bumper.run();
  bumper_msg.print();
  delay(50);
}
