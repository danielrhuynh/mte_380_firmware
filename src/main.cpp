#include <Arduino.h>
#include <stdlib.h>  // for atoi and atof
#include <string.h>  // for strtok
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

// Encoder Pins (if needed)
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
// Servo Object
// -----------------------------------
Servo myServo;

// -----------------------------------
// Encoder Interrupt Service Routines
// -----------------------------------
void IRAM_ATTR leftEncoderPressed() {
  left_ticks++;
}

void IRAM_ATTR rightEncoderPressed() {
  right_ticks++;
}

// -----------------------------------
// Serial Input Handling Variables
// -----------------------------------
String inputString = "";     // String to hold incoming data
bool stringComplete = false; // Flag for completed string

// Helper function to clear the serial bugger on start
void clearSerialBuffer() {
  while(Serial.available() > 0) {
    Serial.read(); // Read and discard each byte
  }

  inputString = "";
  stringComplete = false;
}

// -----------------------------------
// MotorDriver Class Definition
// -----------------------------------
class MotorDriver {
  public:
    int in1Pin, in2Pin, enPin, pwmChannel;
    
    // Constructor: initialize pins and set up PWM (using LEDC)
    MotorDriver(int in1, int in2, int en, int channel)
      : in1Pin(in1), in2Pin(in2), enPin(en), pwmChannel(channel) {
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
      // Configure PWM: 5000 Hz frequency, 8-bit resolution (0-255)
      ledcSetup(pwmChannel, 5000, 8);
      ledcAttachPin(enPin, pwmChannel);
    }
    
    // Run motor at given speed (0-255); direction: true = forward
    void run(int speed, bool forward) {
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

// Instantiate motor driver objects using unique LEDC channels (0 and 1)
MotorDriver motor_left(M1_IN1, M1_IN2, M1_EN, 0);
MotorDriver motor_right(M2_IN1, M2_IN2, M2_EN, 1);

// -----------------------------------
// PID Controller Variables
// -----------------------------------
float prev_error = 0;
float integral_error = 0;
unsigned long last_pid_time = 0;

// -----------------------------------
// Motor Control (PID) Callback Function
// -----------------------------------
// Accepts error value and optional PID & speed parameters.
void motorControlCallback(int error,
                          int base_speed,   // default ~200
                          int min_speed,    // default ~100
                          int max_speed,    // max PWM value 255
                          float Kp,
                          float Ki,
                          float Kd) {
  unsigned long current_time = millis();
  // Calculate dt in seconds
  float dt = (current_time - last_pid_time) / 1000.0;
  last_pid_time = current_time;
  if (dt < 0.001) dt = 0.001;
  
  // Compute PID terms
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
  
  // Apply correction: positive error increases left motor speed, decreases right motor speed.
  float left_speed_f  = base_speed + correction;
  float right_speed_f = base_speed - correction;
  
  // Constrain speeds to [min_speed, max_speed]
  int left_speed  = constrain((int)left_speed_f, min_speed, max_speed);
  int right_speed = constrain((int)right_speed_f, min_speed, max_speed);
  
  // Run both motors forward at the calculated speeds
  bool left_forward_bool = left_speed > 0;
  bool right_forward_bool = right_speed > 0;
  motor_left.run(left_speed, left_forward_bool);
  motor_right.run(right_speed, right_forward_bool);
}

// -----------------------------------
// Turn Robot by Specified Degrees
// -----------------------------------
// Turn control variables
bool turn_in_progress = false;
int turn_target_ticks = 0;
bool turn_clockwise = true;
const float TICKS_PER_DEGREE = 0.4722; // 170 ticks / 360° given by Jackie

// Turn PID variables
float turn_kp = 1.0;
float turn_ki = 0.0;
float turn_kd = 0.05;
float turn_prev_error = 0;
float turn_integral = 0;
unsigned long turn_last_time = 0;
void startTurn(int degrees, bool clockwise = true, float kp = 1.0, float ki = 0.0, float kd = 0.05, float slipping_factor = 1.5) {
  clearSerialBuffer();
  left_ticks = 0;
  right_ticks = 0;
  
  // turn_target_ticks = (degrees * TICKS_PER_DEGREE) / 2;
  turn_target_ticks = (degrees * TICKS_PER_DEGREE * slipping_factor);
  turn_clockwise = clockwise;
  
  turn_kp = kp;
  turn_ki = ki;
  turn_kd = kd;
  
  turn_prev_error = 0;
  turn_integral = 0;
  turn_last_time = millis();
  
  turn_in_progress = true;
  Serial.print("Starting turn of ");
  Serial.print(degrees);
  Serial.print(" degrees with PID: ");
  Serial.print(kp); Serial.print(", ");
  Serial.print(ki); Serial.print(", ");
  Serial.print(kd); Serial.print(", ");
  Serial.println(slipping_factor);
}

void updateTurn() {
  if (!turn_in_progress) return;
  
  // Use average ticks in case one wheel experiences more slipping than the other
  int avg_ticks = (left_ticks + right_ticks) / 2;
  
  if (avg_ticks >= turn_target_ticks) {
    motor_left.stop();
    motor_right.stop();
    turn_in_progress = false;
    
    Serial.print("Turn completed: ");
    Serial.print(avg_ticks);
    Serial.print(" avg ticks (target: ");
    Serial.print(turn_target_ticks);
    Serial.print("), L:");
    Serial.print(left_ticks);
    Serial.print(" R:");
    Serial.println(right_ticks);
    return;
  }
  
  // Add safety timeout
  static unsigned long turn_start_time = 0;
  if (turn_start_time == 0) turn_start_time = millis();
  
  // Stop turn after 10s
  if (millis() - turn_start_time > 10000) {
    motor_left.stop();
    motor_right.stop();
    turn_in_progress = false;
    Serial.println("Turn timeout - stopped at " + String(avg_ticks) + " ticks");
    turn_start_time = 0;
    return;
  }
  
  int error = turn_target_ticks - avg_ticks;
  
  unsigned long current_time = millis();
  float dt = (current_time - turn_last_time) / 1000.0;
  turn_last_time = current_time;
  if (dt < 0.001) dt = 0.001;  // Prevent division by zero
  
  float p_term = turn_kp * error;
  
  turn_integral += error * dt;
  float max_integral = 100;
  if (turn_integral > max_integral) turn_integral = max_integral;
  else if (turn_integral < -max_integral) turn_integral = -max_integral;
  float i_term = turn_ki * turn_integral;
  
  float derivative = (error - turn_prev_error) / dt;
  float d_term = turn_kd * derivative;
  turn_prev_error = error;
  
  // Calculate motor speed based on PID output
  float pid_output = p_term + i_term + d_term;
  int base_speed = 250;
  int turn_speed = constrain(base_speed + (int)pid_output, 80, 255);
  int RIGHT_WHEEL_ADJUSTMENT_FACTOR = 2;
  
  // Apply motor control based on turning direction
  if (turn_clockwise) {
    motor_left.run(turn_speed, true);    // Left motor forward
    motor_right.run(turn_speed * RIGHT_WHEEL_ADJUSTMENT_FACTOR, false);  // Right motor backward
  } else {
    motor_left.run(turn_speed, false);   // Left motor backward
    motor_right.run(turn_speed * RIGHT_WHEEL_ADJUSTMENT_FACTOR, true);   // Right motor forward
  }
  
  // Reset timeout when a new turn starts
  if (!turn_in_progress) turn_start_time = 0;
}

// -----------------------------------
// Servo Functions
// -----------------------------------
int pos = 0;    // variable to store the servo position
void servoClose() {
  // Move servo to 0° (closed position)
  myServo.write(0);
  Serial.println("Servo Closed");
}

void servoOpen() {
  // Move servo to 90° (open position)
  myServo.write(180);
  Serial.println("Servo Opened");
}

// -----------------------------------
// Arduino Setup
// -----------------------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.setRxBufferSize(4096);  // Increase RX buffer size
  Serial.begin(921600);
  
  // Setup encoder pins with internal pull-ups (if using encoders)
  pinMode(ENC_A_LEFT, INPUT_PULLUP); 
  pinMode(ENC_A_RIGHT, INPUT_PULLUP);
  
  // Attach interrupts to encoder pins (trigger on FALLING edge)
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT), leftEncoderPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), rightEncoderPressed, FALLING);
  
  // Initialize PID timing
  last_pid_time = millis();
  
  // Reserve space for incoming serial data
  inputString.reserve(200);
  
  // Setup Servo
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);// Standard 50hz servo
  myServo.attach(SERVO_PIN, 1000, 2000);  // 1000-2000 μs pulse range
  myServo.write(0);  // Start at 0° (closed)
}

// -----------------------------------
// Arduino Main Loop
// -----------------------------------
void loop() {
  // Process serial commands when a complete string is received
  if (stringComplete) {
    inputString.trim();
    
    // Check if command starts with "MC_"
    if (inputString.startsWith("MC_")) {
      String params = inputString.substring(3); // Remove "MC_" prefix
      
      // Default parameters will be overwritten by actual controller
      int errorVal = 0;
      int baseSpeed = 200; // default base_speed
      int minSpeed  = 100; // default min_speed
      int maxSpeed  = 255; // default max_speed
      float kp = 0.1;
      float ki = 0.0;
      float kd = 0.0;
      
      // If no comma, then only error is provided
      if (params.indexOf(',') == -1) {
        errorVal = params.toInt();
      } else {
        // Convert the parameters string to a char array for tokenization
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
      
      // Call the motor control callback with the parsed parameters
      motorControlCallback(errorVal, baseSpeed, minSpeed, maxSpeed, kp, ki, kd);
      
    } else if (inputString.startsWith("OPEN")) {
      // Command to open the servo (move to 90°)
      servoOpen();
    } else if (inputString.startsWith("CLOSE")) {
      // Command to close the servo (move to 0°)
      servoClose();
    } else if (inputString.startsWith("GET_ENC")) {
      // Command to get encoder values for the 180 degree turn
      Serial.print("ENC_");
      Serial.print(left_ticks);
      Serial.print(",");
      Serial.println(right_ticks);
    } else if (inputString.startsWith("TURN_DEG")) {
      int degrees = 180;
      float kp = 1.0;     // Default PID constants
      float ki = 0.0;
      float kd = 0.05;
      bool clockwise = true;
      float slipping_factor = 2.8;
      
      if (inputString.indexOf('_') != -1) {
        // Split by underscores to get parameters
        String parts[5];  // Max 5 parts: TURN_DEG_degrees_kp_ki_kd
        int partCount = 0;
        int startPos = 0;
        int endPos = 0;
        
        while (endPos >= 0 && partCount < 5) {
          endPos = inputString.indexOf('_', startPos);
          if (endPos >= 0) {
            parts[partCount++] = inputString.substring(startPos, endPos);
            startPos = endPos + 1;
          } else {
            parts[partCount++] = inputString.substring(startPos);
          }
        }
        
        // Process parameters based on how many were provided
        if (partCount > 2) {  // At least TURN_DEG_degrees
          degrees = parts[2].toInt();
        }
        
        if (partCount > 3) {  // Has Kp
          kp = parts[3].toFloat();
        }
        
        if (partCount > 4) {  // Has Ki
          ki = parts[4].toFloat();
        }
        
        if (partCount > 5) {  // Has Kd
          kd = parts[5].toFloat();
        }

        if (partCount > 6) {  // Has slip factor
          slipping_factor = parts[6].toFloat();
        }
      }
      
      if (degrees <= 0) degrees = 180;
      
      startTurn(degrees, clockwise, kp, ki, kd, slipping_factor);
    } else {
      Serial.print("Unknown command: ");
      Serial.println(inputString);
    }
    inputString = "";
    stringComplete = false;
    
  }
  updateTurn();
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