#include "clearinghouse.h"

using namespace gw;
//************************************************************************
//*                     CLASS CLEARINGHOUSE
//************************************************************************
Clearinghouse::Clearinghouse()
{
}

void Clearinghouse::register_msg(Message* msg) 
{
	store.push_back(msg);
}

void Clearinghouse::list()
{
	Serial.print("Store contains: \n");
	for(int i = 0; i < store.size(); ++i) {
		Serial.print("\t");
		store[i]->print();
	}
}

void Clearinghouse::update(Message* msg) 
{
	int this_id = msg->id();
	for(int i = 0; i < store.size(); ++i) {
		/* code */
	}
	
}

Message* Clearinghouse::get_ptr(const char* name)
{
	//users must test for NULL before using result
	
	Message* res = NULL;
	//if names match then return ptr to this message
	for(int i = 0; i < store.size(); ++i) {
		const char* str1 = store[i]->name();
		if(!strcmp(str1, name) ) {	//strcmp returns 0 on match
			res = store[i];
		}
	}
	return res;
}

Message* Clearinghouse::get_ptr(const int msg_id)
{
	//users must test for NULL before using result
	
	Message* res = NULL;
	//if id matches then return ptr to this message
	for(int i = 0; i < store.size(); ++i) {
		if(msg_id == store[i]->id() ) {  
//			Serial.println(store[i]->id());
//			Serial.println("found the message id");
			res = store[i];
		}
	}
	return res;
}


