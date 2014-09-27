#include <Servo.h>

#include <Scanner_5pt.h>

#include <Vector.h>

#include <Pair.h>

#include <clearinghouse.h>
#include <messages/five_pt_scan.h>

gw::Clearinghouse ch;

/*------------Set the message count for the system-----------------------*/
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

Five_pt_scan_msg five_pt_scan_msg;

Scanner_5pt scanner(6, 9, 88, 130, (240/60), true);

void setup() {
  Serial.begin(57600);
  Serial.println();
  
  ch.list();
  
  scanner.begin();
  scanner.print();
  
  delay(7500);

}

void loop() {
  scanner.run();
  five_pt_scan_msg.print();  
//  ch.list();
}
