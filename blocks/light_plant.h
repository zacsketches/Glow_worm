#ifndef LIGHT_PLANT_H
#define LIGHT_PLANT_H JUL_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/cmd_led.h>

//gw components

//other 3rd party libraries
#include <Wire.h>

//debug control
#define INCLUDE_LIGHT_PRINT 0

//*******************************************************************
//*                         LIGHT_PLANT
//* Plant configured to drive five LEDs based off the Cmd_led_msg
//*
//* Based off the PCF8575 I2C expander
//*******************************************************************

extern Cmd_led_msg cmd_led_msg;
extern Clearinghouse ch;

class Light_plant : public gw::Node {
private:
    
	//I2C address of PCF8575 I2C expander with A2=H, A1=H, A0=L
	static const int default_address = 0x26;
	
	// If the I2C expander is wired to sink current through the leds
	// instead of sourcing current then you must write a 0 to the led
	// port instead of a 1.  My hardware is put together this way.
	static const bool sink_current = true;
	
    Vector<gw::Led*> leds;
	int nr_of_leds;
	int i2c_address;
    
	//Subscriber and local copy of the message
	gw::Subscriber<Cmd_led_msg> sub;
	Cmd_led_msg local_msg;
	    
	//*** 	HELPER FUNCTIONS ***//
	Port::port get_port(const char* led_name)
	{
		/*
		 * Search the led vector and return the port of the named
		 * led.  If there is not match then return the Port::error
		 * value. 
		*/
		Port::port res = Port::error;
		for(int i = 0; i < leds.size(); ++i) {
			if( !strcmp(led_name, leds[i]->name()) ){
				res = leds[i]->port();
			}
		}
		return res;
	}
		
public:
	
    Light_plant(int number_of_leds =5, int addr =default_address)
        :Node("light_plant"), 
		nr_of_leds(number_of_leds),
		i2c_address(addr),
		sub(&cmd_led_msg, &ch, local_msg)
	{
            leds.reserve(number_of_leds);   //allocate memory for the leds vector
	}
        
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {
		//This block requires I2C, but all Glow Worm components should check
		//the framework variable gw::wire_begun before calling the Wire
		//library function so that it only gets called once across the whole
		//system.
		if(!gw::wire_begun) {
			Wire.begin();
			gw::wire_begun = true;
		}
	}
		
	void attach(gw::Led* led) {
        leds.push_back(led);
	}

	void run()
	{
		sub.update();
		
		byte port_0 = 0x00;
		byte port_1 = 0x00;
		Port::port tmp;
		
		// if local_msg is State::on then
		// turn port_0's corresponding bit for the led on.

		tmp = get_port("far_lt");
		if( (tmp != Port::error) && (local_msg.far_lt == Led_state::on) ) {
			port_0 |= (int)tmp;
		}
		else {
			int inverse = (int)tmp;
			inverse = ~inverse;		//invert the port to shut the led off
			port_0 &= inverse;
		}
		tmp = get_port("near_lt");
		if( (tmp != Port::error) && (local_msg.near_lt == Led_state::on) ) {
			port_0 |= (int)tmp;
		}
		else {
			int inverse = (int)tmp;
			inverse = ~inverse;		//invert the port to shut the led off
			port_0 &= inverse;
		}
		tmp = get_port("mid");
		if( (tmp != Port::error) && (local_msg.mid == Led_state::on) ) {
			port_0 |= (int)tmp;
		}
		else {
			int inverse = (int)tmp;
			inverse = ~inverse;		//invert the port to shut the led off
			port_0 &= inverse;
		}
		tmp = get_port("far_rt");
		if( (tmp != Port::error) && (local_msg.far_rt == Led_state::on) ) {
			port_0 |= (int)tmp;
		}
		else {
			int inverse = (int)tmp;
			inverse = ~inverse;		//invert the port to shut the led off
			port_0 &= inverse;
		}
		tmp = get_port("near_rt");
		if( (tmp != Port::error) && (local_msg.near_rt == Led_state::on) ) {
			port_0 |= (int)tmp;
		}
		else {
			int inverse = (int)tmp;
			inverse = ~inverse;		//invert the port to shut the led off
			port_0 &= inverse;
		}

		
		if(sink_current) {
			port_0 = ~port_0;	//Invert to sink current and turn the leds on
		}
			
		//then write port0 and port1 to the i2c expander
		Wire.beginTransmission(i2c_address);
		Wire.write(port_0);
		Wire.write(port_1);
		Wire.endTransmission();	
	}
	#if INCLUDE_LIGHT_PRINT == 1	
		void print()
		{
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.print(name());   
			Serial.print("\t");
			Serial.print(leds.size());
			Serial.print(F(" leds attached: \n"));
			for(int i = 0; i < leds.size(); ++i) {
				Serial.print(F("\t"));
				leds[i]->print();
			}
			Serial.print(F("\tSubscribed to: "));
			sub.subscribed_where();
		}
	#endif
};

#endif