//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>


//Glow Worm core
#include <clearinghouse.h>

//Glow Worm Messages
//#include <messages/control_effort.h>
//#include <messages/plant_status.h>

//Glow Worm Blocks
//#include <blocks/balance_plant.h>

//other libraries (Servo, Wire, etc)
//#include <Wire.h>
//#include <Servo.h>
//#include <quadrature.h>

//Logging Macros
#define LOG_UART Serial
#define LOG(x) LOG_UART.println(x)
#define LOG_P(x_float, prec) LOG_UART.println(x_float, prec); 

/*------------Required to initialize GW framework------------------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
//Control_effort_msg control_effort_msg;
//Plant_status_msg plant_status_msg;

/*------------Physical Connections--------------------------------------*/
// const int lt_dir_pin = 12;
// const int lt_pwm_pin = 3;
// const int lt_sense_pin = A0;
// const int lt_encoder_A_pin = 7;
// const int lt_encoder_B_pin = 6;


/*------------Construct the system blocks-------------------------------*/
//Motor(name, dir_pin, pwm_pin, current_sense_pin, Position::position)
// gw::Motor* lt_motor = 
//   new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, lt_sense_pin, Position::lt);  

//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
// Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>* lt_encoder = 
//   new Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>(Position::lt,"lt_enc");

// //Blance_plant()
// Balance_plant plant;

/*------------Setup-----------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Serial.println();

//  LOG("test of log macro");

//  ch.list();
  
//  Wire.begin();
//  gw::wire_begun = true;

//  Attach tools to blocks
//  plant.attach(lt_encoder);
//  plant.attach(lt_motor);

//  Start the blocks by calling their .begin() function
//  plant.begin();

//  Inspect blocks by calling their .print() function
//  plant.print();

  delay(2000);
}

/*------------Loop------------------------------------------------------*/
void loop() {
  //Run each block
  //  plant.run();
  
  //Inspect the global messages on each loop if desired
  //  plant_status_msg.print();
}
