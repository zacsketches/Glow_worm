#include <arduino.h>
#include <clearinghouse.h>

/*------------Set the message count for the system-----------------------*/
int Message::msg_count = 0;
int Node::node_count = 0;

/*------------Concrete implementation of Int_msg-------------------------*/
struct Simple_msg : public Message {
  Simple_msg() : Message("simple") {}

  int data;  
  
  //updates this message from the data in the passed message	
  void update(Message* msg) {
    Simple_msg* ptr = static_cast<Simple_msg*>(msg);
    data = ptr->data;
  }
  
  void print() {
	Serial.print(id());
	Serial.print("\t");
	Serial.print(name());
	Serial.print("\t");
	Serial.println(data);
  } 
};

/*------------Concrete implementation of Float_msg-------------------------*/
struct Float_msg : public Message {
  Float_msg() : Message("float") {}

  float data;  

  void update(Message* msg) {
    Float_msg* ptr = static_cast<Float_msg*>(msg);
    data = ptr->data;
  }
  
  void print() {
	Serial.print(id());
	Serial.print("\t");
	Serial.print(name());
	Serial.print("\t");
	Serial.println(data);
  } 
};

/*------------Concrete implementation of a producer----------------------*/
/* We use extern here to get access to the global message and clearinghouse */

extern Simple_msg m1;
extern Float_msg m2;
extern Clearinghouse ch;

class Producer : public Node {
private:

public:
	Producer() : Node("producer"), 
		pub1(&m1, &ch, local_msg1),
		pub2(&m2, &ch, local_msg2)
	{}

	void update_int() {
		pub1.publish();
	}

	Publisher<Simple_msg> pub1;
	Publisher<Float_msg>  pub2;	
	Simple_msg local_msg1;
	Float_msg  local_msg2;
};

/*------------Concrete implementation of a client----------------------*/
class Client : public Node {
private:
	
public:
	Client() : Node("client"),
		sub1(&m1, &ch, local_msg)
	{}
	
	void run() {
		sub1.update();
	}
	
	Subscriber<Simple_msg> sub1;
	Simple_msg local_msg;
};





