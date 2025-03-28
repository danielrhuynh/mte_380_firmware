#include <Arduino.h>
#include <stdlib.h>   // for atoi and atof
#include <string.h>   // for strtok
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// -----------------------------------
// New Pin Definitions for ESP32
// -----------------------------------
// Motor 1 (Left)
const int M1_IN1 = 25;
const int M1_IN2 = 26;
const int M1_EN   = 27;  // PWM via LEDC

// Motor 2 (Right)
const int M2_IN1 = 32;
const int M2_IN2 = 33;
const int M2_EN  = 14;   // PWM via LEDC

// Encoder Pins
const int ENC_A_LEFT  = 4;
const int ENC_A_RIGHT = 5;

// Servo Pin
const int SERVO_PIN = 13;  // Adjust to your wiring

// -----------------------------------
// Global Encoder Tick Counters
// -----------------------------------
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// -----------------------------------
// Ticks Conversion Constants
// -----------------------------------
// For driving straight: 
// Wheel diameter = 65 mm, so circumference ≈ 65 * PI ≈ 204.2 mm,
// and 170 ticks per revolution → TICKS_PER_MM ≈ 170 / 204.2 ≈ 0.832.
const float TICKS_PER_MM = 0.7904;

// -----------------------------------
// Servo Object
// -----------------------------------
Servo myServo;

// -----------------------------------
// MotorDriver Class Definition
// -----------------------------------
class MotorDriver {
  public:
    int in1Pin, in2Pin, enPin, pwmChannel;
    // New flag: true means “forward” so add ticks; false means reverse so subtract ticks.
    volatile bool tick_direction;

    MotorDriver(int in1, int in2, int en, int channel)
      : in1Pin(in1), in2Pin(in2), enPin(en), pwmChannel(channel), tick_direction(true) {
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
      // Configure PWM: 5000 Hz frequency, 8-bit resolution (0-255)
      ledcSetup(pwmChannel, 5000, 8);
      ledcAttachPin(enPin, pwmChannel);
    }
    
    // Run motor at given speed (0-255) with a direction flag.
    // forward==true means “forward” so we add encoder ticks;
    // forward==false means reverse so we subtract ticks.
    void run(int speed, bool forward) {
      tick_direction = forward; // update flag for encoder ticks
      if (forward) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
      } else {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
      }
      ledcWrite(pwmChannel, speed);
    }
    
    // Stop the motor
    void stop() {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      ledcWrite(pwmChannel, 0);
    }
};

// Instantiate motor driver objects using unique LEDC channels
MotorDriver motor_left(M1_IN1, M1_IN2, M1_EN, 0);
MotorDriver motor_right(M2_IN1, M2_IN2, M2_EN, 1);

// -----------------------------------
// Encoder Interrupt Service Routines
// -----------------------------------
void IRAM_ATTR leftEncoderPressed() {
  if (motor_left.tick_direction) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void IRAM_ATTR rightEncoderPressed() {
  if (motor_right.tick_direction) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}

// -----------------------------------
// Existing PID Controller Variables (for straight-line control)
// -----------------------------------
float prev_error = 0;
float integral_error = 0;
unsigned long last_pid_time = 0;

// -----------------------------------
// Motor Control (PID) Callback Function (existing)
// -----------------------------------
void motorControlCallback(int error,
                          int base_speed,   // default ~200
                          int min_speed,    // default ~100
                          int max_speed,    // max PWM value 255
                          float Kp,
                          float Ki,
                          float Kd) {
  unsigned long current_time = millis();
  float dt = (current_time - last_pid_time) / 1000.0;
  last_pid_time = current_time;
  if (dt < 0.001) dt = 0.001;
  
  float p_term = Kp * error;
  integral_error += error * dt;
  float max_integral = 100;
  if (integral_error > max_integral) integral_error = max_integral;
  else if (integral_error < -max_integral) integral_error = -max_integral;
  float i_term = Ki * integral_error;
  float derivative = (error - prev_error) / dt;
  float d_term = Kd * derivative;
  prev_error = error;
  
  float correction = p_term + i_term + d_term;
  
  float left_speed_f  = base_speed + correction;
  float right_speed_f = base_speed - correction;
  
  int left_speed  = constrain((int)left_speed_f, min_speed, max_speed);
  int right_speed = constrain((int)right_speed_f, min_speed, max_speed);
  
  bool left_forward_bool = left_speed > 0;
  bool right_forward_bool = right_speed > 0;
  motor_left.run(left_speed, left_forward_bool);
  motor_right.run(right_speed, right_forward_bool);
}

// -----------------------------------
// PID Controller for Turning
// -----------------------------------
const float TICKS_PER_DEGREE = 1.235; // Example conversion factor for turning

void turnPID(float target_degrees, int max_speed = 255, int min_speed = 100,
             float Kp_diff = 0.5, float Ki_diff = 0.0, float Kd_diff = 0.1,
             float Kp_base = 2.0) {
  left_ticks = 0;
  right_ticks = 0;
  
  float target_ticks = abs(target_degrees) * TICKS_PER_DEGREE;
  
  bool left_forward, right_forward;
  if (target_degrees > 0) {
    left_forward = true;
    right_forward = false;
  } else {
    left_forward = false;
    right_forward = true;
  }
  
  float prev_diff_error = 0;
  float integral_diff = 0;
  unsigned long last_time = millis();
  
  while (((abs(left_ticks) + abs(right_ticks)) / 2.0) < target_ticks) {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    if (dt <= 0) dt = 0.001;
    last_time = current_time;
    
    float diff_error = left_ticks + right_ticks;
    integral_diff += diff_error * dt;
    float derivative_diff = (diff_error - prev_diff_error) / dt;
    prev_diff_error = diff_error;
    float correction = Kp_diff * diff_error + Ki_diff * integral_diff + Kd_diff * derivative_diff;
    
    float avg_ticks = (abs(left_ticks) + abs(right_ticks)) / 2.0;
    float distance_error = target_ticks - avg_ticks;
    
    int base_speed = min_speed + (int)(Kp_base * distance_error);
    if (base_speed > max_speed) base_speed = max_speed;
    if (base_speed < min_speed) base_speed = min_speed;
    
    int left_speed, right_speed;
    if (target_degrees > 0) { // Clockwise: left forward, right backward
      left_speed = base_speed - (int)correction;
      right_speed = base_speed + (int)correction;
    } else { // Counter-clockwise: left backward, right forward
      left_speed = base_speed + (int)correction;
      right_speed = base_speed - (int)correction;
    }
    
    left_speed = constrain(left_speed, min_speed, max_speed);
    right_speed = constrain(right_speed, min_speed, max_speed);
    
    motor_left.run(left_speed, left_forward);
    motor_right.run(right_speed, right_forward);
    
    delay(10);
  }
  
  motor_left.stop();
  motor_right.stop();
}

// -----------------------------------
// PID Controller for Driving Straight
// -----------------------------------
// This function drives the robot straight for a desired distance (in mm).
// It uses one PID loop to correct the difference between the two wheels (error = left_ticks - right_ticks)
// and a second PID loop to set the base speed based on the distance error.
// PID Controller for Driving Straight (Modified for Negative Distances)
void driveStraightPID(float target_distance, int max_speed = 255, int min_speed = 100,
  float Kp_diff = 0.5, float Ki_diff = 0.0, float Kd_diff = 0.1,
  float Kp_base = 2.0) {
// Reset encoder ticks
left_ticks = 0;
right_ticks = 0;

// Determine drive direction: forward if target_distance >= 0, backwards otherwise.
bool drive_forward = (target_distance >= 0);

// Compute the target tick count (use absolute value)
float target_ticks = fabs(target_distance) * TICKS_PER_MM;

// PID variables for the wheel difference correction.
float prev_diff_error = 0;
float integral_diff = 0;
unsigned long last_time = millis();

while (((abs(left_ticks) + abs(right_ticks)) / 2.0) < target_ticks) {
unsigned long current_time = millis();
float dt = (current_time - last_time) / 1000.0;
if (dt <= 0) dt = 0.001;
last_time = current_time;

// Difference error: ideally both encoders should count equally.
float diff_error = left_ticks - right_ticks;
integral_diff += diff_error * dt;
float derivative_diff = (diff_error - prev_diff_error) / dt;
prev_diff_error = diff_error;
float correction = Kp_diff * diff_error + Ki_diff * integral_diff + Kd_diff * derivative_diff;

// Distance error based on the average of the absolute encoder ticks.
float avg_ticks = (abs(left_ticks) + abs(right_ticks)) / 2.0;
float distance_error = target_ticks - avg_ticks;

// Calculate a base speed that decreases as the robot nears the target distance.
int base_speed = min_speed + (int)(Kp_base * distance_error);
if (base_speed > max_speed) base_speed = max_speed;
if (base_speed < min_speed) base_speed = min_speed;

// Adjust motor speeds using the correction.
// If left wheel is ahead (diff_error > 0), slow left and speed up right.
int left_speed = base_speed - (int)correction;
int right_speed = base_speed + (int)correction;
if (!drive_forward) {
  left_speed = base_speed + (int)correction;
  right_speed = base_speed - (int)correction;
}

left_speed = constrain(left_speed, min_speed, max_speed);
right_speed = constrain(right_speed, min_speed, max_speed);

// Run both motors in the same direction:
// 'true' for forward, 'false' for reverse.
motor_left.run(left_speed, drive_forward);
motor_right.run(right_speed, drive_forward);

delay(10);
}

motor_left.stop();
motor_right.stop();
}


// -----------------------------------
// Servo Functions (unchanged)
// -----------------------------------
int pos = 0;
void servoClose() {
  if (!myServo.attached()) {
    myServo.attach(SERVO_PIN, 1000, 2000);
    delay(100);
  }
  myServo.write(0);
  delay(1000);
  myServo.detach();
  Serial.println("Servo Closed");
}

void servoOpen() {
  if (!myServo.attached()) {
    myServo.attach(SERVO_PIN, 1000, 2000);
    delay(100);
  }
  myServo.write(180);
  delay(1000);
  myServo.detach();
  Serial.println("Servo Opened");
}

// -----------------------------------
// Serial Input Handling Variables
// -----------------------------------
String inputString = "";
bool stringComplete = false;

// -----------------------------------
// Arduino Setup
// -----------------------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.setRxBufferSize(4096);
  Serial.begin(921600);
  
  pinMode(ENC_A_LEFT, INPUT_PULLUP); 
  pinMode(ENC_A_RIGHT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT), leftEncoderPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), rightEncoderPressed, FALLING);
  
  last_pid_time = millis();
  inputString.reserve(200);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);
}

// -----------------------------------
// Arduino Main Loop
// -----------------------------------
void loop() {
  if (stringComplete) {
    inputString.trim();
    
    if (inputString.startsWith("MC_")) {
      String params = inputString.substring(3);
      int errorVal = 0;
      int baseSpeed = 200;
      int minSpeed  = 100;
      int maxSpeed  = 255;
      float kp = 0.1;
      float ki = 0.0;
      float kd = 0.0;
      
      if (params.indexOf(',') == -1) {
        errorVal = params.toInt();
      } else {
        char buf[100];
        params.toCharArray(buf, 100);
        char *token = strtok(buf, ",");
        int tokenIndex = 0;
        while (token != NULL) {
          switch(tokenIndex) {
            case 0:
              errorVal = atoi(token);
              break;
            case 1:
              baseSpeed = atoi(token);
              break;
            case 2:
              minSpeed = atoi(token);
              break;
            case 3:
              maxSpeed = atoi(token);
              break;
            case 4:
              kp = atof(token);
              break;
            case 5:
              ki = atof(token);
              break;
            case 6:
              kd = atof(token);
              break;
            default:
              break;
          }
          token = strtok(NULL, ",");
          tokenIndex++;
        }
      }
      
      motorControlCallback(errorVal, baseSpeed, minSpeed, maxSpeed, kp, ki, kd);
      
    } else if (inputString.startsWith("TURN")) {
      int commaIndex = inputString.indexOf(',');
      float target_degrees = 0;
      int max_speed = 255;
      int min_speed = 100;
      float Kp_diff = 0.5;
      float Ki_diff = 0.0;
      float Kd_diff = 0.1;
      float Kp_base = 2.0;
      
      if (commaIndex > 0) {
        String params = inputString.substring(commaIndex + 1);
        char buf[100];
        params.toCharArray(buf, 100);
        char *token = strtok(buf, ",");
        int tokenIndex = 0;
        while(token != NULL) {
          switch(tokenIndex) {
            case 0:
              target_degrees = atof(token);
              break;
            case 1:
              max_speed = atoi(token);
              break;
            case 2:
              min_speed = atoi(token);
              break;
            case 3:
              Kp_diff = atof(token);
              break;
            case 4:
              Ki_diff = atof(token);
              break;
            case 5:
              Kd_diff = atof(token);
              break;
            case 6:
              Kp_base = atof(token);
              break;
            default:
              break;
          }
          token = strtok(NULL, ",");
          tokenIndex++;
        }
      }
      
      turnPID(target_degrees, max_speed, min_speed, Kp_diff, Ki_diff, Kd_diff, Kp_base);
      Serial.print("COMPLETE");
    } else if (inputString.startsWith("DRIVE")) {
      // Serial command example:
      // "DRIVE,1000" or "DRIVE,1000,255,100,0.5,0,0.1,2.0"
      // where parameters are:
      // target_distance (mm), max_speed, min_speed, Kp_diff, Ki_diff, Kd_diff, Kp_base
      int commaIndex = inputString.indexOf(',');
      float target_distance = 0;
      int max_speed = 255;
      int min_speed = 100;
      float Kp_diff = 0.5;
      float Ki_diff = 0.0;
      float Kd_diff = 0.1;
      float Kp_base = 2.0;
      
      if (commaIndex > 0) {
        String params = inputString.substring(commaIndex + 1);
        char buf[100];
        params.toCharArray(buf, 100);
        char *token = strtok(buf, ",");
        int tokenIndex = 0;
        while(token != NULL) {
          switch(tokenIndex) {
            case 0:
              target_distance = atof(token);
              break;
            case 1:
              max_speed = atoi(token);
              break;
            case 2:
              min_speed = atoi(token);
              break;
            case 3:
              Kp_diff = atof(token);
              break;
            case 4:
              Ki_diff = atof(token);
              break;
            case 5:
              Kd_diff = atof(token);
              break;
            case 6:
              Kp_base = atof(token);
              break;
            default:
              break;
          }
          token = strtok(NULL, ",");
          tokenIndex++;
        }
      }
      
      driveStraightPID(target_distance, max_speed, min_speed, Kp_diff, Ki_diff, Kd_diff, Kp_base);
      Serial.print("COMPLETE");
      
    } else if (inputString.startsWith("GET_ENC")) {
      Serial.print("Left ticks: ");
      Serial.print(left_ticks);
      Serial.print(", Right ticks: ");
      Serial.println(right_ticks);
      
    } else if (inputString.startsWith("OPEN")) {
      servoOpen();
    } else if (inputString.startsWith("CLOSE")) {
      servoClose();
    }
      else if (inputString.startsWith("STOP")) {
      motor_left.stop();
      motor_right.stop();
      motor_left.run(100, !motor_left.tick_direction);
      motor_right.run(100, !motor_right.tick_direction);
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(inputString);
    }
    if((left_ticks + right_ticks) / 2 > 25000){
      Serial.println("PASS");
    }
    
    inputString = "";
    stringComplete = false;
  }
}

// -----------------------------------
// SerialEvent: Called when new data is available on Serial
// -----------------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
