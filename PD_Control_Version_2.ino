//5/12 Attempt of using linear weights for associating sensor inputs to error. 
//Note: this does not implement the idea of precalibrating the input
#include <ECE3.h>

uint16_t sensorValues[8];


//Unused from previous version: 
//#define INDEX_TO_MM(idx) (((idx) - 3.5f) * (80.0f / 7.0f))

//Motor Pins
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;


//Error Constants
const float error_weights[8] = {0.004593909641, -0.00230461646641071, -0.00598458978362412, -0.0101981729036942, -0.0307421424067153, -0.0207302076484164, -0.0190907385628999, -0.0345495210837488};
const float error_constant = 107.568325052908;
const float all_black_threshold = 140.000;

//PD Constants
float Kp = 1.1;     
float Kd = 0.6;     

int baseSpeed = 80;
int maxSpeed  = 180;

float previousError = 0.0;

//Turning 
enum RobotState {
  FOLLOWING,
  TURNING,
  RESUMING       // brief pause after turn to let sensors re-acquire line
};

RobotState state = FOLLOWING;

unsigned long turnStartTime  = 0;
unsigned long resumeStartTime = 0;

// TURN_DURATION — time in ms for a 180-degree spin at TURN_SPEED
#define TURN_SPEED    90
#define TURN_DURATION 625     // ms — adjust until 180 degree rotation done successfully
#define RESUME_PAUSE  200     // ms — short settle time before PD resumes


float computeError() {
  ECE3_read_IR(sensorValues);
  float sum_error = 0;
  for(int i = 0; i < 8; i++) {
    float individual_sensor_error = sensorValues[i]*error_weights[i];
    sum_error += individual_sensor_error;
  }
  //This is the final error
  return sum_error + error_constant;
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

}

void loop() {
  // put your main code here, to run repeatedly:
  if (state == TURNING) {
    if (millis() - turnStartTime >= TURN_DURATION) {
      setMotorSpeeds(0,0); //This stops the car 
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
      state = FOLLOWING;
    }
    return;
  }

  // --- STATE: FOLLOWING ---
  float error_mm = computeError();

  if (error_mm >= all_black_threshold) {
    // First detection — confirm it's real, not a fake reading
    float second_error_mm = computeError();

    if (second_error_mm >= all_black_threshold) {
      // Confirmed end-of-track line: start the turn
      turnStartTime = millis();
      setMotorSpeeds(TURN_SPEED, -TURN_SPEED); // spin in place
      state = TURNING;
    }
    // If second read didn't confirm, fall through and keep following
    return;
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
