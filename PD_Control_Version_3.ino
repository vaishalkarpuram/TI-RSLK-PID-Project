//5/26 Version 3B 
#include <ECE3.h>


//Motor Pins
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;


const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;


uint16_t sensorValues[8];
const uint16_t WHITE_MIN[8] = {700, 700, 700, 700, 700, 700, 700, 700}; 
const uint16_t BLACK_MAX[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};


//Create a time variable that measures the time between the first and second rotation of black. and then you only allow the stop if the sufficient time has passed between (travelling backwards through the track). so it is a failsafe against the scenario of turning, overshooting due to high-speed inertia, and then turning again.
//float first_rotation_detection_time = 0.0f;
//float second_rotation_detection_time = 0.0f;
//float time_between_rotations = 0.0f;
//The second rotation is when the car is looking to stop.


//const float ALLOWED_TIME_DURATION_BETWEEN_SUCCESSIVE_ROTATIONS = 5000.0f; //equivalent to 5 seconds
const float STRAIGHT_TIME_AFTER_ROTATION = 250.0f;
//This can be made into 300f.


//This gives the normalization floats that are used to transform sensor outputs into a 0 to 1000 output range.
float IR_SCALE[8];
// Call this once in setup() to compute the scale factors
void calibrateScales() {
 for (int i = 0; i < 8; i++) {
   int range = BLACK_MAX[i] - WHITE_MIN[i];
   if (range <= 0) range = 1; // guards against bad constants
   IR_SCALE[i] = 1000.0f / range;
 }
}


//These are the linear weights applied to the normalized sensor outputs.
//Linear weights from raw google sheets
//const float ERROR_WEIGHTS[8] = {0.008334096785, -0.00480769960939605, -0.0114848435733688, -0.0207167513707849, -0.0628494294686115, -0.0388155743101453, -0.0335687205023411, -0.0623156438317299};
//Attempt to make linear weights symmetric
const float ERROR_WEIGHTS[8] = {0.08, 0.058, 0.038, 0.0257, -0.0252, -0.04, -0.058, -0.08};
const float ERROR_CONSTANT =  25.0f;


//This sets the threshold that 6 of the 8 sensors must read an all black or the cumulative sensor output must be greater than 6000
#define ALL_BLACK_THRESHOLD 6000.0f


//PD Constants
float Kp = 1.38;    //1.38 and 14 gives 14.9
float Kd = 14.00;    


int baseSpeed = 190;
int maxSpeed  = 300;


float previousError = 0.0;


//Turning
enum RobotState {
 FOLLOWING,
 TURNING,
 RESUMING       // brief pause after turn to let sensors re-acquire line
};
int NUM_ROTATIONS = 0;


RobotState state = FOLLOWING;


unsigned long turnStartTime  = 0;
unsigned long resumeStartTime = 0;
unsigned long initialStartTime = 0;


// TURN_DURATION — time in ms for a 180-degree spin at TURN_SPEED
#define TURN_SPEED    255
#define TURN_DURATION 250     // ms — adjust until 180 degree rotation done successfully
#define RESUME_PAUSE  200     // ms — short settle time before PD resumes


#define START_STRAIGHT_TIME 300 //the initial moving forward so that the positions 3 and 4 are easier to start PID on.


//Error Function


float computeError() {
 ECE3_read_IR(sensorValues);


 float fusedError  = 0.0f;
 float sumCalib    = 0.0f;


 for (int i = 0; i < 8; i++) {
   // Step 1: subtract white baseline, clamp to 0
   int shifted = (int)sensorValues[i] - WHITE_MIN[i];
   if (shifted < 0) shifted = 0;


   // Step 2: scale to [0, 1000]
   float calibrated = shifted * IR_SCALE[i];
   if (calibrated > 1000.0f) calibrated = 1000.0f;


   // Step 3: accumulate weighted error and total activation
   fusedError += calibrated * ERROR_WEIGHTS[i];
   sumCalib   += calibrated;
 }


 // Step 4: detect all-black crossbar before adding constant
 if (sumCalib >= ALL_BLACK_THRESHOLD) {
   return 100000.0f; // sentinel — triggers turn logic in loop()
 }


 // Step 5: add bias constant → final error in mm
 return fusedError + ERROR_CONSTANT;
}




void setLeftMotor(int speed) {
 speed = constrain(speed, -maxSpeed, maxSpeed);


 if (speed >= 0) {
   digitalWrite(left_dir_pin, LOW);
   analogWrite(left_pwm_pin, speed);
 } else {
   digitalWrite(left_dir_pin, HIGH);
   analogWrite(left_pwm_pin, -speed);
 }
}


void setRightMotor(int speed) {
 speed = constrain(speed, -maxSpeed, maxSpeed);


 if (speed >= 0) {
   digitalWrite(right_dir_pin, LOW);
   analogWrite(right_pwm_pin, speed);
 } else {
   digitalWrite(right_dir_pin, HIGH);
   analogWrite(right_pwm_pin, -speed);
 }
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {
 setLeftMotor(leftSpeed);
 setRightMotor(rightSpeed);
}






void setup() {
 // put your setup code here, to run once:
 ECE3_Init();
 calibrateScales();
 Serial.begin(9600);


 pinMode(left_nslp_pin, OUTPUT);
 pinMode(left_dir_pin, OUTPUT);
 pinMode(left_pwm_pin, OUTPUT);


 pinMode(right_nslp_pin, OUTPUT);
 pinMode(right_dir_pin, OUTPUT);
 pinMode(right_pwm_pin, OUTPUT);


 digitalWrite(left_nslp_pin, HIGH);
 digitalWrite(right_nslp_pin, HIGH);


 delay(2000);
 initialStartTime = millis();
 setMotorSpeeds(50, 50);
 while(millis() - initialStartTime <= START_STRAIGHT_TIME) {
    //This allows the car to move a tiny bit before starting PID.
   //This ensures the starting position before PID is optimal (the sensors are able to detect the black line below.)
 }


}


void loop() {
 // put your main code here, to run repeatedly:
 if (state == TURNING) {
   if (millis() - turnStartTime >= TURN_DURATION) {
     setMotorSpeeds(0,0); //This stops the car
     delay(250);
     resumeStartTime = millis();
     state = RESUMING;
   }
   // else: motors are already spinning from when we entered TURNING, do nothing
   return;
 }


 // --- STATE: RESUMING (brief settle after turn) ---
 if (state == RESUMING) {
   if (millis() - resumeStartTime >= RESUME_PAUSE) {
     previousError = 0.0; // reset derivative — stale error would spike the correction
     //Set the motor speed to base speed and let it run for 100 milliseconds to stop second rotation detection and stopping halfway
     setMotorSpeeds(50, 50);
     unsigned long startTime = millis();
     //Causes the car to move straight for the specified amount of time.
     while(millis() - startTime < STRAIGHT_TIME_AFTER_ROTATION){


     }
     state = FOLLOWING;
   }
   return;
 }


 // --- STATE: FOLLOWING ---
 float error_mm = computeError();


 if (error_mm >= ALL_BLACK_THRESHOLD) {
   // First detection — confirm it's real, not a fake reading
   float second_error_mm = computeError();


   //Trying to limit the inertia (at high speed, the car detects black but overshoots, turns and then sees the same black again and stops at milestone 12)
   //setMotorSpeeds(0,0);
   //Trying to limit the inertia (at high speed, the car detects black but overshoots, turns and then sees the same black again and stops at milestone 12)
   //setMotorSpeeds(0,0);
   if(state != TURNING && error_mm>=ALL_BLACK_THRESHOLD) {
   if (second_error_mm >= ALL_BLACK_THRESHOLD && NUM_ROTATIONS < 1) {
     // Confirmed we have hit the end of the track: start the turn
     setMotorSpeeds(0,0);
     delay(250);
     turnStartTime = millis();
     setMotorSpeeds(TURN_SPEED, -TURN_SPEED); // spin in place
     NUM_ROTATIONS = 1;
     state = TURNING;
   }else if(second_error_mm >= ALL_BLACK_THRESHOLD && NUM_ROTATIONS >= 1){
     setMotorSpeeds(0,0); //This stops the car.
     delay(15000); //This is equivalent to stopping the car at the end of the run.
   }
     // If second read didn't confirm, fall through and keep following
     return;
   }
 }


 //Regular PD control algorithm
 float derivative = error_mm - previousError;


 float correction = Kp * error_mm + Kd * derivative;


 int leftSpeed  = baseSpeed + correction;
 int rightSpeed = baseSpeed - correction;


 leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
 rightSpeed = constrain(rightSpeed, 0, maxSpeed);


 setMotorSpeeds(leftSpeed, rightSpeed);


 previousError = error_mm;




}
