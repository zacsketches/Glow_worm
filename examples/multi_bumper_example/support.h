#include <clearinghouse.h>
#include <messages/bumper.h>

extern Bumper_msg lt_local_bumper_msg;
extern Bumper_msg rt_local_bumper_msg;
extern Bump_state::bs b_state;

inline Bump_state::bs calc_b_state() {
    Bump_state::bs res = Bump_state::clear;
	Serial.print("LEFT message is:\n\t");
	lt_local_bumper_msg.print();
	Serial.print("RIGHT message is:\n\t");
	rt_local_bumper_msg.print();
    if (lt_local_bumper_msg.pressed) res = Bump_state::lt_bump;
    if (rt_local_bumper_msg.pressed) res = Bump_state::rt_bump;
    
    return res;
}

inline void show_bs() {
    Serial.print(F("Bumper state is: "));
    Serial.println(text(b_state));
}
