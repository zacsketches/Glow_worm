/* 
    Due to static friction every motor has a dead band where
    the control effort is too small to move the motor.  See this
    link for a good discussion of deadband:
    
    http://claymore.engineer.gvsu.edu/~jackh/eod/courses/egr345/fall05/lab_deadband.pdf
    
    Here we slowly ramp the control effort up until we get a velocity
    on the wheels.  We print to serial on each iteration assuming the
    system is connected to the serial port while running the experiment.
    
    Each loop the control effort is increased by the step_size defined
    in the global variable below.  Adjust this variable for a finer tuned
    deadband solution.  After each new step the system delays for the amount
    of step_delay defined below which allows the motor to run at that step 
    for a moment before proceeding to the next step.

	The system will compute the number of loops defined in loops and 
	report an average of those values at the end of the test.
    
    This dead band solver is set up to work with the balance plant which
    consists of two quadrature encoded motors and is driven from the
    control effort message.  A simpler version could be set up where the
    motor is driven directly from PWM commands and you manually record 
    when the motor starts to turn.
*/

int step_size = 50;
const int step_delay = 1500;
const int starting_effort = 0;
const int loops = 5;

//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//Glow Worm Messages
#include <messages/control_effort.h>
#include <messages/plant_status.h>

//Glow Worm Blocks
#include <blocks/balance_plant.h>

//other libraries (Servo, Wire, etc)
#include <quadrature.h>

//Logging Macros
#define LOG_UART Serial
#define LOG(x) LOG_UART.println(x)
#define LOG_P(x_float, prec) LOG_UART.println(x_float, prec); 

//***********************************************************************
//                               Velocity Computer
//***********************************************************************
class Velocity_computer {
private:
    //local data
    double x;
    double x_dot;
    
    double last_x;
    double last_x_dot;
    
    Time last_x_time;
    Time x_time;

public:
    Velocity_computer() {}
    
    void reset(double new_last_x, double new_last_x_time){
        last_x = new_last_x;
        last_x_time = new_last_x_time;
    }
    
    double solve(double measured_x, Time measured_t) {
        //compute x
        x = measured_x;
        
        //compute x_dot
        x_time = measured_t;
        Time x_dt = x_time - last_x_time;
        double delta_x = x - last_x;
        x_dot = delta_x / x_dt;
        
        //shift the variables
        last_x = x;
        last_x_time = x_time;

        return x_dot;
    }
};

//************************************************************************
//*               Find Average of the vals in vector<int>
//************************************************************************
int find_avg(Vector<int>& vec) {
	int sum = 0;
	for(int i = 0; i < vec.size(); ++i) {
		sum += vec[i];
	}
	int avg = sum / vec.size();
	
	return avg;
}


//***********************************************************************
//                               Main Sketch
//***********************************************************************

/*------------Required to initialize GW framework------------------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Control_effort_msg control_effort_msg;
Plant_status_msg plant_status_msg;

/*------------Physical Connections--------------------------------------*/
const int lt_dir_pin = 12;
const int lt_pwm_pin = 3;
const int lt_sense_pin = A0;
const int lt_encoder_A_pin = 7;
const int lt_encoder_B_pin = 6;

const int rt_dir_pin = 13;
const int rt_pwm_pin = 11;
const int rt_sense_pin = A1;
const int rt_encoder_A_pin = 5;
const int rt_encoder_B_pin = 4;

/*------------Construct the system blocks-------------------------------*/
//Motor(name, dir_pin, pwm_pin, current_sense_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, lt_sense_pin, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", rt_dir_pin, rt_pwm_pin, rt_sense_pin, Position::rt);  
//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>* lt_encoder = 
  new Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>(Position::lt,"lt_enc");
Quadrature_encoder<rt_encoder_A_pin,rt_encoder_B_pin>* rt_encoder = 
  new Quadrature_encoder<rt_encoder_A_pin,rt_encoder_B_pin>(Position::rt,"rt_enc");
//Blance_plant()
Balance_plant plant;

//Unique class to compute the velocity of the motor
Velocity_computer computer;

/*------------Setup-----------------------------------------------------*/
void setup() {
    Serial.begin(115200);
    Serial.println();

    LOG("test of log macro");

    ch.register_msg(&control_effort_msg);

    ch.list();

    //initialize the balance plant
    rt_motor->reverse();        //right motor is installed opposite left
  //  lt_encoder->reverse();      //left motor encoder installed opposite
    plant.attach(lt_encoder);
    plant.attach(rt_encoder);
    plant.attach(lt_motor);
    plant.attach(rt_motor);
    plant.begin();
    plant.print();
    
    //set initial conditions..this will update the plant_status_message to
    //the current values for the two encoders..should be zero, but in some
    //cases it may be a small int close to zero.
    control_effort_msg.u = 0;
    plant.run();
    

    delay(2000);
}

/*------------Loop------------------------------------------------------*/
void loop() {
    static int effort = 0;
    static bool test_complete = false; 
    static Time now = 0;
    static Time step_complete = 0;
	static Vector<int> rt_fwd, rt_back, lt_fwd, lt_back;       
    
	for(int i = 0; i < loops; ++i) {
		Serial.print("\t\t*****BEGINNING LOOP ");
		Serial.print(i);
		Serial.println("*****");
	    //compute deadband for right motor going forward
	    LOG("Testing right motor in the forward direction.");
	    effort = starting_effort;
	    test_complete = false;
	    now = millis();
	    computer.reset(plant_status_msg.rt_ct, plant_status_msg.timestamp);
	    while (!test_complete) {
	        //set up the step
	        now = millis();
	        step_complete = now + step_delay;
	        control_effort_msg.u = effort;
        
	        //run the step
	        while(now < step_complete) {
	            plant.run();            
	            now = millis();
	        }
        
	        //compute the velocity
	        double v_x = computer.solve(plant_status_msg.rt_ct, plant_status_msg.timestamp);

	        //report the results
	        Serial.print("\teffort is:");
	        Serial.print(effort);
	        Serial.print("\tVelocity is:");
	        Serial.println(v_x, 2);
        
	        //check to see if the step was sufficient
	        if(abs(v_x) >= .05) {
	          test_complete = true;
	          Serial.print("\tRight motor began turning at an effort of: ");
	          Serial.println(effort);
	        }
	        //get ready for the next loop
	        effort += step_size;
	    }

	    //complete the first test
		rt_fwd.push_back(effort - step_size);
	    LOG("Completed the right forward test");
	    control_effort_msg.u = 0;
	    plant.run();
	    delay(1000);
	    plant.run();
    
	    //compute deadband for right motor going backward
	    LOG("Testing right motor in the backward direction.");
	    effort = -starting_effort;
	    test_complete = false;
	    now = millis();
	    computer.reset(plant_status_msg.rt_ct, plant_status_msg.timestamp);
	    while (!test_complete) {
	        //set up the step
	        now = millis();
	        step_complete = now + step_delay;
	        control_effort_msg.u = effort;
        
	        //run the step
	        while(now < step_complete) {
	            plant.run();            
	            now = millis();
	        }
        
	        //compute the velocity
	        double v_x = computer.solve(plant_status_msg.rt_ct, plant_status_msg.timestamp);

	        //report the results
	        Serial.print("\teffort is:");
	        Serial.print(effort);
	        Serial.print("\tVelocity is:");
	        Serial.println(v_x, 2);
        
	        //check to see if the step was sufficient
	        if(abs(v_x) >= .05) {
	          test_complete = true;
	          Serial.print("\tRight motor began turning at an effort of: ");
	          Serial.println(effort);
	        }        
	        //get ready for the next loop
	        effort -= step_size;
	    }

	    //complete the second test
		rt_back.push_back(effort - step_size);
	    LOG("Completed the right backward test");
	    control_effort_msg.u = 0;
	    plant.run();
	    delay(1000);
	    plant.run();

	    //compute deadband for left motor going forward
	    LOG("Testing left motor in the forward direction.");
	    effort = starting_effort;
	    test_complete = false;
	    now = millis();
	    computer.reset(plant_status_msg.lt_ct, plant_status_msg.timestamp);
	    while (!test_complete) {
	        //set up the step
	        now = millis();
	        step_complete = now + step_delay;
	        control_effort_msg.u = effort;
        
	        //run the step
	        while(now < step_complete) {
	            plant.run();            
	            now = millis();
	        }
        
	        //compute the velocity
	        double v_x = computer.solve(plant_status_msg.lt_ct, plant_status_msg.timestamp);

	        //report the results
	        Serial.print("\teffort is:");
	        Serial.print(effort);
	        Serial.print("\tVelocity is:");
	        Serial.println(v_x, 2);
        
	        //check to see if the step was sufficient
	        if(abs(v_x) >= .05) {
	          test_complete = true;
	          Serial.print("\tLeft motor began turning at an effort of: ");
	          Serial.println(effort);
	        }
	        //get ready for the next loop
	        effort += step_size;
	    }

	    //complete the third test
		lt_fwd.push_back(effort - step_size);
	    LOG("Completed the left forward test");
	    control_effort_msg.u = 0;
	    plant.run();
	    delay(1000);
	    plant.run();
    
	    //compute deadband for left motor going backward
	    LOG("Testing left motor in the backward direction.");
	    effort = -starting_effort;
	    test_complete = false;
	    now = millis();
	    computer.reset(plant_status_msg.lt_ct, plant_status_msg.timestamp);
	    while (!test_complete) {
	        //set up the step
	        now = millis();
	        step_complete = now + step_delay;
	        control_effort_msg.u = effort;
        
	        //run the step
	        while(now < step_complete) {
	            plant.run();            
	            now = millis();
	        }
        
	        //compute the velocity
	        double v_x = computer.solve(plant_status_msg.lt_ct, plant_status_msg.timestamp);

	        //report the results
	        Serial.print("\teffort is:");
	        Serial.print(effort);
	        Serial.print("\tVelocity is:");
	        Serial.println(v_x, 2);
        
	        //check to see if the step was sufficient
	        if(abs(v_x) >= .05) {
	          test_complete = true;
	          Serial.print("\tLeft motor began turning at an effort of: ");
	          Serial.println(effort);
	        }        
	        //get ready for the next loop
	        effort -= step_size;
	    }

	    //complete the fourth test
		lt_back.push_back(effort - step_size);
	    LOG("Completed the left backward test");
	    control_effort_msg.u = 0;
	    plant.run();
	    delay(1000);
	    plant.run();  
	}
	
	//display the averages
	int avg_rt_fwd = find_avg(rt_fwd);
	int avg_rt_back = find_avg(rt_back);
	int avg_lt_fwd = find_avg(lt_fwd);
	int avg_lt_back = find_avg(lt_back);
	Serial.print("The rt fwd avg is: ");
	Serial.println(avg_rt_fwd);
	Serial.print("The rt back avg is: ");
	Serial.println(avg_rt_back);
	Serial.print("The lt fwd avg is: ");
	Serial.println(avg_lt_fwd);
	Serial.print("The lt back avg is: ");
	Serial.println(avg_lt_back);
	
    //infinite loop to stop execution
    while(1) {
      control_effort_msg.u = 0;
      plant.run(); 
    }
}







