#ifndef CMD_VELOCITY_H
#define CMD_VELOCITY_H SEP_2014

#include <arduino.h>
#include <clearinghouse.h>

#define INCLUDE_PRINT 1

//************************************************************************
//*                         COMMAND VELOCITY MESSAGE
//************************************************************************

using namespace gw;

struct Cmd_velocity_msg : public Message {
	
	// Data available from base class
	//      const char* name()
	//      int id()
	char* tag;
	Direction::dir l_dir;
	int l_spd;
	Direction::dir r_dir;
	int r_spd;
    
    //minimal constructor
    Cmd_velocity_msg() : Message("cmd_velocity"),
		l_dir(Direction::fwd), l_spd(0), r_dir(Direction::fwd), r_spd(0),
		tag(" ")
	{}
	
	//data and default name constructor
	Cmd_velocity_msg(Direction::dir lt_dir,
	    int lt_spd,
	    Direction::dir rt_dir,
	    int rt_spd) 
	    : Message("cmd_velocity"), l_dir(lt_dir), l_spd(lt_spd),
	    r_dir(rt_dir), r_spd(rt_spd), tag(" ")
	{}

	//full constructor
	Cmd_velocity_msg( char* vec_tag,
		Direction::dir lt_dir,
	    int lt_spd,
	    Direction::dir rt_dir,
	    int rt_spd) 
	    : Message("cmd_Velocity"), tag(vec_tag), l_dir(lt_dir), l_spd(lt_spd),
	    r_dir(rt_dir), r_spd(rt_spd)
	{}
    
	void update(Message* msg) {
		Cmd_velocity_msg* ptr = static_cast<Cmd_velocity_msg*>(msg);
		tag = ptr->tag;
        l_dir = ptr->l_dir;
        l_spd = ptr->l_spd;
        r_dir = ptr->r_dir;
        r_spd = ptr->r_spd;		
	}
	
	void update(const Message* msg) {
		Message* non_const_msg = const_cast<Message*>(msg);
		update(non_const_msg);
	}

   #if INCLUDE_PRINT == 1
    void print() {
        char buf[100];
        sprintf(buf, "{id:%d, name:%s, tag:%s, L_dir:%s, L_spd:%d, R_dir:%s, R_spd:%d}",
            id(), name(), tag, Direction::text(l_dir), l_spd, Direction::text(r_dir), r_spd);
        Serial.println(buf);
    }
    
    const void print() const {
        char buf[100];
        sprintf(buf, "{id:%d, name:%s, tag:%s, L_dir:%s, L_spd:%d, R_dir:%s, R_spd:%d}",
            id(), name(), tag, Direction::text(l_dir), l_spd, Direction::text(r_dir), r_spd);
        Serial.println(buf);
    }
	#endif
};

#endif
