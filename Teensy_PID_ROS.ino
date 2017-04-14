#include <ros.h>
#include <sensor_msgs/Joy.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <WProgram.h>
#include <Sabertooth.h>

/* Angle to counts conversion parameters */
#define STEERING_ANGLE_MAX      15                 // Steering Angle, degrees
#define POT_ANGLE_MAX           165                // Potentiometer Angle, degrees
#define POT_GEAR_RATIO          POT_ANGLE_MAX/STEERING_ANGLE_MAX
#define RADIANS_TO_ANGLE        57.3               // 360/2PI
#define ADC_COUNTS              1024               // 10 bits
#define ADC_OFFSET              ADC_COUNTS/2
#define POT_ADC_GAIN            ADC_COUNTS/(2*POT_ANGLE_MAX)

/* Max Feedback limits. DO NOT EXCEED the AtoD range 0 - 1024 */
#define MAX_RIGHT               924
#define MAX_LEFT                308

/* Servo parameters */
#define DELTA_T                 .001      // .001 seconds
#define SAMPLE_TIME             1         // Sample time = 1/DELTA_T in (ms)
#define ERROR_FILTER_FREQ       50        // 50Hz
#define FILTER_SCALE_UP         128       // Required for integer arithmetic
#define ERROR_FILTER_SCALE      6.28*DELTA_T*ERROR_FILTER_FREQ*FILTER_SCALE_UP
#define SETPOINT_FILTER_FREQ    10        // Hz
#define SETPOINT_FILTER_SCALE   6.28*DELTA_T*SETPOINT_FILTER_FREQ*FILTER_SCALE_UP
#define ST_MAX                  127
#define OUTPUT_RANGE            256           // 2*ST_MAX + 1
#define DEAD_BAND               10            // We need this because of gear backlash noise
#define OUTPUT_DESCALE          256           
/*  OUTPUT_DESCALER is required for using integer arithmetic 
 *  Since we are using an AtoD the error signal is scaled up by the ADC_COUNT value
 *  When we add Kp, Ki, and Kd we futher increase the output control signal.
 *  This will then require a de-scaling to a level compatible with the output signal level.
 * Typical starting value = ADC_COUNTS*PROPORTIONAL_GAIN/OUTPUT_RANGE. 
 * Adjust as necssary.  This value is also dependent on the signal clipping limits.
 */

typedef short int16;
typedef int   int32;

ros::NodeHandle nh; //define ROS node handle

/* PID Parameters */
#define PROPORTIONAL_GAIN        64            // K16p
#define DIFFERENTIAL_GAIN        128           // Kd
#define INTEGRAL_GAIN            0             // Ki = 0, integral is not used to much gear lash

/*Throttle mapping range */
#define THROTTLE_IN_MIN           1
#define THROTTLE_IN_MAX          -1
#define THROTTLE_OUT_MIN          0
#define THROTTLE_OUT_MAX          50


//PID, feedback and filtering variables
int32  proportional, differential, out = 0;
int32  errorFilterDot, errorFilterOut = 0, errorFilter = 0;
int32  setpointFilterDot, setpointFilterOut = 0, setpointFilter = 0;
int16  feedback, setpoint, error, lastError = 0;

Sabertooth ST(128, Serial1);

// ROS joy function using Logitech F310 USB gamepad
void joydata(const sensor_msgs::Joy& joy)
{
  double dJoy, throttle;
  bool   orange, red, reverse; //orange trim left, red trim righ, hold green for reverse
  static double cal = 100;

  /*Steering setpoint*/
  orange = joy.buttons[3];
  red = joy.buttons[1];
  reverse = joy.buttons[0];
  
  if(orange) //orange button sets trim let
    cal+=20;
  if(red)
    cal-=20; //red button sets trim right
  dJoy = joy.axes[0];//left stick, steering
  dJoy*= ADC_OFFSET;
  dJoy+= ADC_OFFSET + cal;
  if(dJoy>MAX_RIGHT) //max right
    dJoy = MAX_RIGHT;
  else if(dJoy < MAX_LEFT) //max left
    dJoy = MAX_LEFT;
  setpoint = (int16)dJoy;

  /*Throttle*/ //right trigger
  throttle = int(THROTTLE_OUT_MIN + (((joy.axes[5]-THROTTLE_IN_MIN)*(THROTTLE_OUT_MAX-THROTTLE_OUT_MIN))
          / (THROTTLE_IN_MAX-THROTTLE_IN_MIN))); //mapping function throttle in as -1 through 1 to 0 through 127
  
  //reverse, hold green button for reverse
  if(reverse == true)
  {
    ST.motor(2, -throttle);
  }
  else if (reverse == false)
  {
    if(throttle == 0)
      ST.motor(2, 0);
    if(throttle > 0);
      ST.motor(2, throttle);  
  }
}

//ROS joy subscriber
ros::Subscriber<sensor_msgs::Joy> sub("/joy", joydata);

void setup()
{                
  //Serial.begin(9600);
  SabertoothTXPinSerial.begin(9600);
  Serial1.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(A0, INPUT_PULLUP);
  feedback = analogRead(0); // Get current position
  setpoint = feedback;      // Start out from previous setpoint
  errorFilterOut = 0;       // Initialize the filter integrators
  errorFilter = errorFilterOut*FILTER_SCALE_UP;
  setpointFilterOut = setpoint;
  setpointFilter = FILTER_SCALE_UP*setpointFilterOut;
  nh.initNode();
  nh.subscribe(sub);
}

void loop()                     
{
  digitalWrite(3, HIGH);   // turn the LED on
/*Arduino style debug*/
// testing setpoint with serial monitor
//  if(Serial.peek()>0)
//  setpoint = Serial.parseInt();
//  cal = Serial.parseInt();
  /* Filtering the setpoint slows the setpoint response */
  setpointFilterDot = SETPOINT_FILTER_SCALE*(setpoint - setpointFilterOut); // Apply filter coefficent & scale
  setpointFilter+= setpointFilterDot;                                       // Integrate the derivative
  setpointFilterOut = setpointFilter/FILTER_SCALE_UP;                       // Descale
  feedback = analogRead(0);
  if(feedback < MAX_RIGHT && feedback > MAX_LEFT)
  { // If this is false it will indicate a runaway motor if we lose out feedback
    error = setpointFilterOut - feedback;  // Servo error signal
    /* Filtering the error signal helps remove derivative noise */
    errorFilterDot = ERROR_FILTER_SCALE*(error - errorFilterOut); // Apply filter coefficent
    errorFilter+= errorFilterDot;                                 // Integrate the error signal derivative
    errorFilterOut = errorFilter/FILTER_SCALE_UP;
    proportional = PROPORTIONAL_GAIN*errorFilterOut;
    differential = DIFFERENTIAL_GAIN*errorFilterDot;
    differential/= FILTER_SCALE_UP;
    // integral+= INTEGRAL_GAIN*errorFilterOut; Not used for this application (gearing friction/hystersis)
    out = proportional + differential;          // out = proportional + differential + integral;
    out/= OUTPUT_DESCALE;
    // Limit the motor control values
    if(out > ST_MAX)
      out = ST_MAX;
    else if(out < -ST_MAX)
      out = -ST_MAX;
    // Set a dead band to quite gear-lash
    if(out<DEAD_BAND && out>-DEAD_BAND)
      out = 0; // Deadband
    digitalWrite(3, LOW); // This is here for determining the loop service time
    delay(SAMPLE_TIME);   // Delay is used to guarantee a fixed sample rate for the PID
  
/*Arduino style debug*/
//  Serial.print(" feedback = ");
//  Serial.print(feedback);
//  Serial.print(" setpointFilterOut = ");
//  Serial.print(setpointFilterOut);
//  Serial.print(" error = ");
//  Serial.print(error);
//  Serial.print(" errorFilterOut = " );
//  Serial.print(errorFilterOut);
//  Serial.print(" out = ");
//  Serial.print(out);
//  Serial.println();
//  ST.motor(1, cal);
    ST.motor(1, out);
    nh.spinOnce();
  }
  else
    ST.motor(1, 0); // Stop the motor something broke
}

