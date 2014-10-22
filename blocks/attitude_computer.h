#ifndef ATTITUDE_COMPUTER_H
#define ATTITUDE_COMPUTER_H OCT_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/pitch.h>

//gw components
#include <LSM303.h>
#include <L3G.h>

//other libraries
#include <Wire.h>

//debug control
#define INCLUDE_ATTITUDE_PRINT 1

/*
	Integrate a gyro and correct it with an error signal from an
	accelerometer to publish the pitch message.
	
    accel.enable default() results in the following sensor config:
      50hz
      All axes enabled (Xen, Yen, Zen)
      +/- 2g full scale
      Normal mode (vice low-power mode)
      High resolution enabled
      
    This configuration results in a conversion factor of 
    
    1 mG      G
    ----- X -------  X Reading = Val in G's
    LSB     1000 mG

    Gyro calibration:

                      250 deg 
   At sensitivity of  ------- the scaler to convert to deg/sec (datasheet pg9) 
                       1 sec                                               
      8.750 mdeg/sec      1 deg/sec         Reading unit
   is -------------  X ---------------  X ---------------             
          1 unit        1000 mdeg/sec

*/

/*
	TODO consider moving constant to static class data
*/
//Constants
#define RAD_TO_DEG 57.2958  		// 180 / pi
#define DEG_TO_RAD .0174533 		// pi / 180
#define GYRO_COUNTS_TO_DEG .00875   // see above
#define DEG_TO_GYRO_COUNTS 114.2857	// 1 / .00875
#define TIMESCALE 1000.0            // use mills() for timing
//#define TIMESCALE 1000000.0       // use mircos() for timing

//*****************************************************************************
//                               DATA STRUCTURES
//*****************************************************************************

struct Gyro_reading {
    double    omega;
    Time timestamp;
};

struct Error {
    double epsilon;
    Time timestamp;
};

//************************************************************************
//*                         ATTITUDE COMPUTER
//* Control the interface with a Pololu 10DOF IMU v2.  Using my own 
//* algorithm transform this data into a drift compensated gyro input
//* and publish the pitch message
//************************************************************************

// external variables declared in the main sketch
extern Pitch_msg pitch_msg;
extern Clearinghouse ch;

class Attitude_computer : public gw::Node {

	typedef gw::vector<float> vector;

public:
	Attitude_computer(L3G& gyroscope, LSM303& accelerometer) 
		: Node("Att_computer"), now(0),
		calibrated_gyro(gyroscope), 
		calibrated_accel(accelerometer), 
		gyro_plant(calibrated_gyro),
		pub(&pitch_msg, &ch, local_msg)
		
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {
		//make sure Wire.begin() has been called 
		if(gw::wire_begun == false){
			Wire.begin();
			gw::wire_begun = true;
		}
		
		// configure the PI controller
		pi_controller.set_kp(default_kp);
		pi_controller.set_ki(default_ki);
		
	    while(!calibrated_gyro.begin());
		
	    while(!calibrated_accel.begin());
		
		// set initial conditions
	    double initial_condition = calibrated_accel.pitch_angle();
	    theta_gyro = initial_condition;
	    theta_accel = initial_condition;
	}
	
	void set_kp(float kp) {pi_controller.set_kp(kp);}
	void set_ki(float ki) {pi_controller.set_ki(ki);}
	float kp() {return pi_controller.kp();}
	float ki() {return pi_controller.ki();}
	
	/*
		TODO change the constructor to attach the gyro and 
		accel instead of declaring it in construction
	*/
	//     void attach(Motor* m) {
	//         motors.push_back(m);
	//     }
	//
	// void attach(Encoder* n){
	// 	encoders.push_back(n);
	// }
    
	void run() {
		
	    //start from error signal and work clockwise around model...
		//see the model diagram in the datasheets sections for 
		//a diagram of this process
    
	    //run the error computer
	    theta_accel = calibrated_accel.pitch_angle();
	    error_signal = error_computer.run(theta_gyro, theta_accel);
    
	    //run the pi_controller
	    omega_adjustment = pi_controller.run(error_signal);
    
	    //run the drift adjuster
	    omega_raw = calibrated_gyro.read();
	    omega_compensated = drift_adjuster.run(omega_raw, omega_adjustment);
    
	    //run the gyro plant
	    theta_gyro = gyro_plant.run(omega_compensated);		
		
		//update local_msg
		local_msg.timestamp = omega_compensated.timestamp;
		local_msg.theta = theta_gyro;
		
		//publish the pitch message
		pub.publish();
	}
	
private:
	
	Time now;
	
	//inputs to error computer
	double theta_gyro;
	double theta_accel;
	
	//inputs to PI controller
	Error error_signal; 
	
	//inputs to drift adjuster
	Gyro_reading omega_raw;
	double omega_adjustment;
	
	//inputs to gyro plant
	Gyro_reading omega_compensated;

  	//Publisher/Subscribers and local copies
	gw::Publisher<Pitch_msg> pub;
	Pitch_msg local_msg;
		
	// PI tuning based on an hour of tuning 10/16, but needs A LOT more
	// engineering analysis for frequency, and step response with 
	// an evaluation of stability and steady state error.
	static const double default_kp = 5.0;
	static const double default_ki = 0.0;
	
	static const double timescale = TIMESCALE;
	
	//*****************************************************************************
	//                               MODEL COMPONENTS
	//*****************************************************************************

	class Calibrated_gyro {
	public:
	    Calibrated_gyro(L3G& gyroscope) : gyro(gyroscope) {}
    
	    bool begin() {
	        bool result = false;
	        while(!gyro.init());
	        result = true;
	        gyro.enableDefault();
	        return result;
	    }
    
	    Gyro_reading read(){
	        Gyro_reading theta_raw;
	        gyro.read();
	        theta_raw.omega = gyro.G.y-dc_offset;
	        theta_raw.timestamp = millis();
        
	        return theta_raw;
	    }
    
	    double conv_factor() const { return gyro_sensitivity; }
    
	private:
	    L3G& gyro;
	    static const double gyro_sensitivity = .00875;    //datasheet page 9.  mdeg/sec
	    static const long   dc_offset = 126;       //units
          
	} calibrated_gyro;

	class Calibrated_accel {
	public:
	    Calibrated_accel(LSM303& accelerometer) : accel(accelerometer) {}
    
	    bool begin() {
	        bool result = false;
	        while(!accel.init());
	        result = true;
	        accel.enableDefault();
	        return result;
	    }

	    double pitch_angle() {
	        double theta_accel = 0.0;
	        accel.read();
	        accel.shift_accel();
	        float Gx = accel.A.x;
	        float Gy = accel.A.y;
	        float Gz = accel.A.z; 
	        double tan_theta = -Gx / sqrt(pow(Gy, 2) + pow(Gz, 2));

	        theta_accel = atan(tan_theta);
	        theta_accel *= RAD_TO_DEG;

	        return theta_accel - theta_bias;
	    }
    
	private:
	    LSM303& accel;
	    static const float theta_bias =  -0.62306;
	} calibrated_accel;

	class Drift_adjuster {
	public:
	    Drift_adjuster() {}
    
	    Gyro_reading run(Gyro_reading omega_raw, double omega_adjustment) {
	        Gyro_reading omega_compensated;
	        omega_compensated.omega = omega_raw.omega + omega_adjustment;
	        omega_compensated.timestamp = omega_raw.timestamp;
	        return omega_compensated;
	    }
	} drift_adjuster;

	class Gyro_plant {
	public:
	    Gyro_plant(Calibrated_gyro& gyroscope) : gyro(gyroscope)  {}
    
	    double run(Gyro_reading omega_compensated) {
	        //time management
	        Time interval = omega_compensated.timestamp - timestamp_prev;
	        double dt = (double)interval;
	        dt = dt / timescale;
	        timestamp_prev = omega_compensated.timestamp;
        
	        //find trapezoidal omega for integration
	        double omega_mean = (omega_prev + omega_compensated.omega) / 2.0;
        
	        //perform numerical integration
	        theta_counts += omega_mean * dt;
        
	        //get ready for the next run
	        omega_prev = omega_mean;
    
	        theta_angle = theta_counts * gyro.conv_factor();      
  
	        return theta_angle;
	    }
    
	private:
	    Calibrated_gyro& gyro;
	    double theta_counts;
	    double theta_angle;
	    double omega_prev;
	    Time timestamp_prev;
    
	} gyro_plant;

	class Error_computer{
	public:
	    Error_computer() {}
    
	    Error run(double theta_gyro, double theta_accel) {
	        Error result;
	        result.epsilon = theta_gyro - theta_accel;
			result.epsilon *= DEG_TO_GYRO_COUNTS;
			result.epsilon *= -1;
	        result.timestamp = millis();
	        return result;
	    }
	} error_computer;

	class PI_controller {
	public:
	    PI_controller(double kp=0.0, double ki=0.0) :Kp(kp), Ki(ki) {}

	    double run(Error input){
	        double omega_adjustment = 0.0;
        
	        // Time management
	        Time dt = input.timestamp - timestamp_prev;
	        dt /= timescale;
        
	        //integrate
	        epsilon_integrated += input.epsilon * dt;
        
	        //solve for adjustment
	        omega_adjustment = Kp * (input.epsilon) + Ki * (epsilon_integrated);
        
	        return omega_adjustment;
	    }
		
		void set_kp(float kp) {Kp = kp;}
		void set_ki(float ki) {Ki = ki;}
		float kp() {return Kp;}
		float ki() {return Ki;}
    
	private:
	    double Kp;
	    double Ki;
	    Time timestamp_prev;    
	    double epsilon_integrated;
	} pi_controller;
	
};
#endif







