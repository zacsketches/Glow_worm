#ifndef CMD_LED_H
#define CMD_LED_H OCT_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         CMD LED MESSAGE
//************************************************************************

struct Cmd_led_msg : public gw::Message {

	// Data available from base class
	//      const char* name()
	//      int id()
	
    Led_state::led_state far_lt;
    Led_state::led_state near_lt;
    Led_state::led_state mid;
    Led_state::led_state near_rt;
    Led_state::led_state far_rt;
	
	//Constructor
	Cmd_led_msg() : Message("cmd_led"),
		far_rt(Led_state::off),
		near_rt(Led_state::off),
		mid(Led_state::off),
		far_lt(Led_state::off),
		near_lt(Led_state::off)
	{}
		
	//REQUIRED VIRTUAL VOID FROM BASE
	void update(Message* msg) {
		Cmd_led_msg* ptr = static_cast<Cmd_led_msg*>(msg);
	    far_lt = ptr->far_lt;
	    near_lt = ptr->near_lt;
	    mid = ptr->mid;
	    near_rt = ptr->near_rt;
	    far_rt = ptr->far_rt;
	}	
	
	#if INCLUDE_PRINT == 1
	//print
	void print() {
	    char buf[70];
	    sprintf(buf, "{id:%d, name:%s, state:%s, %s, %s, %s, %s}",
	        id(), name(), text(far_lt), text(near_lt), text(mid), text(near_rt), text(far_rt) );
	    Serial.println(buf);
	}
	#endif
};


#endif