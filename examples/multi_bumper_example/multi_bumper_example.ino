#include "support.h"

#include <Vector.h>
#include <Pair.h>

#include <clearinghouse.h>

#include <multi_bumper.h>

#include <messages/bumper.h>

gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

Bumper_msg lt_local_bumper_msg;
Bumper_msg rt_local_bumper_msg;

Multi_bumper b1(12, &lt_local_bumper_msg, &ch, true);
Multi_bumper b2(13, &rt_local_bumper_msg, &ch, true);

Bump_state::bs b_state;

void setup() {
  Serial.begin(57600);
  Serial.println();
  
  b_state = Bump_state::clear;
  
  ch.list();
  b1.begin();
  b2.begin();
  
  b1.print();
  b2.print();
  
  //run them in test and look at their output...there should be a
  //random pattern of activity

}

void loop() {
  Serial.println(F("-----------------------------"));
  b1.run();
  b2.run();
  b1.print();
  b2.print();
  
  b_state = calc_b_state();
  show_bs();
  
  delay(1000);
}
