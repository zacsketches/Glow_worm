#ifndef CMD_VELOCITY_H
#define CMD_VELOCITY_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>

//************************************************************************
//*                         COMMAND VELOCITY MESSAGE
//************************************************************************
#define INCLUDE_PRINT 1

struct Cmd_velocity_msg : public Message {
	
	// Data available from base class
	//      const char* name()
	//      int id()
	
	Direction::dir l_dir;
	int l_spd;
	Direction::dir r_dir;
	int r_spd;
    
    //minimal constructor
    Cmd_velocity_msg() : Message("Cmd_velocity"),
		l_dir(Direction::fwd), l_spd(0), r_dir(Direction::fwd), r_spd(0) 
	{}
    
	void update(Message* msg) {
		Cmd_velocity_msg* ptr = static_cast<Cmd_velocity_msg*>(msg);
        l_dir = ptr->l_dir;
        l_spd = ptr->l_spd;
        r_dir = ptr->r_dir;
        r_spd = ptr->r_spd;		
	}

   #if INCLUDE_PRINT == 1
    void print() {
        char buf[80];
        sprintf(buf, "\t{id: %d, name: %s, l_dir: %s, l_spd: %d, r_dir: %s, r_spd: %d}",
            id(), name(), Direction::text(l_dir), l_spd, Direction::text(r_dir), r_spd);
        Serial.println(buf);
    }
	#endif
};
#endif
 