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
(c) Zac Staples zacsketches (at) github.
You can use this software as you see fit provided you 
acknowledge the source.

    Integrate a gyro and correct it with an error signal from an
    accelerometer to publish the pitch message.

    Use accelerometer pitch reading as negative feedback to the gyro 
    pitch estimation with the intent of reducing the gyro drift error.

    In Dec 2014, I updated the integration technique in this sketch
    to use a custom adaptation of Simpson's rule to perform the 
    numerical gyro integration instead of the simpler trapezoidal
    integration.  My custom implementation of the Simpson rule
    developed with Dirk Lundgren takes the 3/8 Simpson rule for integrting
    over four datapoints and subtracts the standard 1/3 Simpson rule
    for integrating over 3 points to give a result equal only to the dt width
    slice from the 3rd to the 4th datapoint.  This slice is is accumulated
    with each incoming datapoint to produce the gyro estimate.  See the two
    links below for more on these two rules and the wikipedia page on
    Simpsons rule for another explanation.  I have not found a reference that
    analyzes the 4pt - 3pt technique I'm using here, but from the data I 
    got in the gyro_drift_regulator.ino sketch in the MaryLou dev_sketches
    folder I like the results this gives.
    
    http://mathworld.wolfram.com/Simpsons38Rule.html
    http://mathworld.wolfram.com/SimpsonsRule.html
    
    I included a snapshot of the derivation for our use of Simpson's rule
    in the references folder of the MaryLou bot.  See the parts in the
    red annotated loops.
    
    In order to implement the Simpson integration I had to add a data
    structure to the class for a First In First Out list.

    See the PowerPoint brief in the datasheets section for the block
    diagram implemented here.

    DC_offset and Theta_bias from config file.
    Values for gyro DC_offset and accel theta_bias were taken from
    values collected in gyro_test.ino and accel_test.ino run multiple
    times.  These values are a function of installation alignment and 
    inherent sensor error.  
	
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

//Constants
#define GYRO_COUNTS_TO_DEG .00875   // see above
#define DEG_TO_GYRO_COUNTS 114.2857	// 1 / .00875
#define TIMESCALE 1000.0            // use mills() for timing
//#define TIMESCALE 1000000.0       // use mircos() for timing

//provide default constants if there is no included robot config file.
#ifndef THETA_BIAS
    #define THETA_BIAS -0.43329    //FROM EXPERIMENTAL DATA WITH ACCEL_TEST.INO
#endif
#ifndef DC_OFFSET
    #define DC_OFFSET -223         //From experimental data with gyro_test.ino
#endif
#ifndef PITCH_AXIS
    #define PITCH_AXIS y    
#endif
#ifndef ROLL_AXIS
    #define ROLL_AXIS x
#endif
#ifndef YAW_AXIS
    #define YAW_AXIS z
#endif
#ifndef RAD_TO_DEG
    #define RAD_TO_DEG 57.2958
#endif
#ifndef DEG_TO_RAD
    #define DEG_TO_RAD .0174533
#endif

//*****************************************************************************
//                               DATA STRUCTURES
//*****************************************************************************

typedef double Omega;       //Alias for angular velocity
typedef double Epsilon;     //Alias for error
typedef double Angle;       //Alias for angles

struct Gyro_reading {
    Omega    omega;
    Time timestamp;
};

struct Error {
    Epsilon epsilon;
    Time timestamp;
};

template<class T>
struct Fifo_list{
    T list[len];
    int len;
    int pointer;  //keep the pointer on the most recent data

    Fifo_list(int list_length) 
        : len(list_length), pointer(0)
    { }

    void add(const T val) {
           //the pointer locates the position in the array where new data
           //gets inserted
        advance_pointer(pointer);
        list[pointer] = val;
    }

    void advance_pointer(int& ptr) {
        ++ptr;
        if(ptr >= len) {
            ptr = 0;
        }   
    }

    void regress_pointer(int& ptr) {
        --ptr;
        if(ptr < 0) {
            ptr = len-1;
        }   
    }

    void data(T* copy) {
       //fill the copy list with the current data.  I want a list that
       //has the oldest data in position[0] with progressively newer
       //data in subsequent locations
        int dup_pointer = pointer;
        for(int i=len-1; i<0; --i) {
            copy[i]=list[dup_pointer];
            regress_pointer(dup_pointer);   
        }
    }

    double w0() {
        int tmp_ptr = pointer;
        regress_pointer(tmp_ptr);
        regress_pointer(tmp_ptr);
        regress_pointer(tmp_ptr);
        return list[tmp_ptr].omega;
    }

    double w1() {
        int tmp_ptr = pointer;
        regress_pointer(tmp_ptr);
        regress_pointer(tmp_ptr);
        return list[tmp_ptr].omega;
    }

    double w2() {
        int tmp_ptr = pointer;
        regress_pointer(tmp_ptr);
        return list[tmp_ptr].omega;
    }
    double w3() {
        return list[pointer].omega;
    }
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
		// configure the PI controller
		pi_controller.set_kp(default_kp);
		pi_controller.set_ki(default_ki);
		
	    while(!calibrated_gyro.begin());
		
	    while(!calibrated_accel.begin());
		
		// set initial conditions
	    Angle initial_condition = calibrated_accel.pitch_angle();
	    theta_gyro = initial_condition;
	    theta_accel = initial_condition;
	}
	
	void set_kp(const float kp) {pi_controller.set_kp(kp);}
	void set_ki(const float ki) {pi_controller.set_ki(ki);}
	float kp() const {return pi_controller.kp();}
	float ki() const {return pi_controller.ki();}
	
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
	Angle theta_gyro;
    Angle theta_accel;
	
	//inputs to PI controller
	Error error_signal; 
	
	//inputs to drift adjuster
	Gyro_reading omega_raw;
	Epsilon omega_adjustment;
	
	//inputs to gyro plant
	Gyro_reading omega_compensated;

  	//Publisher/Subscribers and local copies
	gw::Publisher<Pitch_msg> pub;
	Pitch_msg local_msg;
		
	// Default values here represent a first hack to get things running.
	// The robot should tune the gains to get the most stable response
	// possible from the attitude computer.
	static const double default_kp = 5.0;
	static const double default_ki = 0.0;
	
	static const double timescale = TIMESCALE;
	
	//*****************************************************************************
	//                               MODEL COMPONENTS
	//*****************************************************************************

	class Calibrated_gyro {
	public:
	    Calibrated_gyro(L3G& gyroscope) 
	        : gyro(gyroscope), 
	        pitch_axis(gyro.G.PITCH_AXIS) 
	    {}
    
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
	        theta_raw.omega = pitch_axis - dc_offset;
	        theta_raw.timestamp = millis();
        
	        return theta_raw;
	    }
    
	    double conv_factor() const { return gyro_sensitivity; }
    
	private:
	    L3G& gyro;
	    static const double gyro_sensitivity = GYRO_COUNTS_TO_DEG;    //datasheet page 9.  mdeg/sec
	    static const long   dc_offset = DC_OFFSET;
        float& pitch_axis;
          
	} calibrated_gyro;

	class Calibrated_accel {
	public:
	    Calibrated_accel(LSM303& accelerometer) 
	        : accel(accelerometer),
	        roll_axis(accel.A.ROLL_AXIS),
	        yaw_axix(accel.A.YAW_AXIS)
	    {}
    
	    bool begin() {
	        bool result = false;
	        while(!accel.init());
	        result = true;
	        accel.enableDefault();
	        return result;
	    }

	    Angle pitch_angle() {
	        Angle theta_accel = 0.0;
	        accel.read();
	        accel.shift_accel();
	        float Gy = roll_axis;
	        float Gz = yaw_axis; 
	        
	        //use atan2 here so the full solution is solved in the range
	        //[-180:180].  Since this attitude computer solves for rotation in
	        //only one axis we don't need to worry about deciding which axis to 
	        //constrain in the [-90:90] realm to eliminate the singularities
	        //of the arctangent.  See datasheets and refs on this for more info.
            theta_accel = atan2(Gy, Gz);

	        theta_accel *= RAD_TO_DEG;
	        return theta_accel - theta_bias;
	    }
    
	private:
	    LSM303& accel;
	    static const Angle theta_bias = THETA_BIAS;
        int16_t& roll_axis;     //using int16_t type because it's used in the LSM303 datasheet.
        int16_t& yaw_axis;
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
    
        Angle run(Gyro_reading omega_compensated) {
            static double theta_counts;
            double omega_prev;

            Angle theta_angle = 0.0;
    	        	    
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

        Angle run_simpson(Gyro_reading omega_compensated) {
            static Fifo_list hist<Omega>(4);
            double simpson_counts;
            static double simpson_counts_prev;
            
            Angle simpson_angle = 0.0;

            hist.add(omega_compensated.omega);

            //time management
            Time interval = omega_compensated.timestamp - timestamp_prev;
            double dt = (double)interval;
            dt = dt / timescale;
            timestamp_prev = omega_compensated.timestamp;

            //would be more accurate if I ignored the first four samples
            simpson_counts = simpson_counts_prev + (9.0/24.0*hist.w3() + 19.0/24.0*hist.w2() - 5.0/24.0*hist.w1() + 1.0/24.0*hist.w0()) * dt;

            simpson_angle = simpson_counts * gyro.conv_factor();

            //get ready for the next run
            simpson_counts_prev = simpson_counts;

            return simpson_angle;
        }

	private:
	    Calibrated_gyro& gyro;
	    Time timestamp_prev;
    
	} gyro_plant;

	class Error_computer{
	public:
	    Error_computer() {}
    
	    Error run(Angle theta_gyro, Angle theta_accel) {
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

        double run_parallel(Error input){
            double omega_adjustment = 0.0;

            // Time management
            Time interval = input.timestamp - timestamp_prev;
            double dt = (double)interval;
            dt = dt / timescale;
            timestamp_prev = input.timestamp;

            //trap z integration for epsilon
            epsilon_integrated += input.epsilon * dt;

            //solve for adjustment
            omega_adjustment = Kp * (input.epsilon) + Ki * (epsilon_integrated);

            return omega_adjustment;
        }

        double run_standard(Error input) {
            //see http://en.wikipedia.org/wiki/PID_controller for a discussion of standard form
            //Ki is related to Ti by the expression
            //         Kp
            //  Ki = -------
            //         Ti
            //
            //therefore, Ki/Kp = 1/Ti
            static double one_over_Ti = Ki/Kp;

            Epsilon omega_adjustment = 0.0;

            // Time management
            Time interval = input.timestamp - timestamp_prev;
            double dt = (double)interval;
            dt = dt / timescale;
            timestamp_prev = input.timestamp;

            //trap z integration for epsilon
            epsilon_integrated += input.epsilon * dt;

            //solve for adjustment
            omega_adjustment = Kp * (input.epsilon +  one_over_Ti*epsilon_integrated);

            return omega_adjustment;  
        }
        
		void set_kp(const float kp) {Kp = kp;}
		void set_ki(const float ki) {Ki = ki;}
		float kp() const {return Kp;}
		float ki() const {return Ki;}
    
	private:
	    double Kp;
	    double Ki;
	    Time timestamp_prev;    
	    Epsilon epsilon_integrated;
	} pi_controller;
	
};
#endif







