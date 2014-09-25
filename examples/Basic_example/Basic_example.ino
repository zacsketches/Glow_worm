#include <Vector.h>
#include <clearinghouse.h>
#include "basic_example.h"

Clearinghouse ch;

Simple_msg m1;
Float_msg  m2;

Producer p_node;

Client s_node;

void setup() {
  Serial.begin(57600);
  Serial.println();
  ch.list();
  
  //test the publish method in the Publisher
  p_node.local_msg1.data = 3;
  p_node.local_msg2.data = 7.685;
  p_node.pub1.publish();
  p_node.pub2.publish();
  Serial.print("p_nodes local int message is: \n\t");
  p_node.local_msg1.print();  
  Serial.print("p_nodes local float message is: \n\t");
  p_node.local_msg2.print();
  Serial.print("After publishing ");
  ch.list();
  
  //test the update method in the Subscriber
  Serial.print("s_nodes local int message is: \n\t");
  s_node.local_msg.print(); 
  s_node.sub1.update();
  Serial.print("After sub.update() s_nodes local int message is: \n\t");
  s_node.local_msg.print();
}

void loop() {
  delay(10); 
}
