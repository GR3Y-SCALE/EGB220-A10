#include <Arduino.h>

// Define sensor pins(from Right to Left) using #define
#define s1 A3
#define s2 A2
#define s3 A1
#define s4 A0
#define s5 A8
#define s6 A7
#define s7 A11
#define s8 A6

// Motor Pins
#define motorLeftDir 7      // Direction pin for left motor
#define motorLeftPWM 3      // PWM pin for left motor speed
#define motorRightDir 17     // Direction pin for right motor
#define motorRightPWM 11    // PWM pin for right motor speed

// PID constants
float Kp = 15.0;  
float Ki = 0.05; 
float Kd = 0.5;  

// Variables for PID
float previousError = 0;
float integral = 0;

// Motor speed PWM
int motorSpeed = 50;

// Sensor values
int sValues[8];
int threshold = 300; // Anything below this value is white


// Function definitions, so we can keep the structure of setup(), loop() up top, makes things a bit cleaner.
void readsensors();
int calculateError(int sValues[]);
void PIDMotorControl(int error);


void setup() {
  // Initialize IR sensor pins as INPUT
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);
  
  // Initialize Left and Right Motor 
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);

  Serial.begin(9600);  // Start serial communication for debugging
}

void loop() {
  readsensors();

  // Print sensor readings
  for (int i = 0; i < 8; i++) {
    Serial.print("S ");
    Serial.print(i + 1);  // Display sensor number (1 to 8)
    Serial.print(": ");
    Serial.println(sValues[i]);
  }

  // Calculate error based on the sensor readings
  int error = calculateError(sValues);

  // Print the error for debugging
  Serial.print("Error: ");
  Serial.println(error);

  // Apply PID control to adjust motor speeds
  PIDMotorControl(error);

  delay(1);
}

void readsensors(){
  // Read values from the IR sensors using defined pin names and store them to sValues[]
  sValues[0] = analogRead(s8);
  sValues[1] = analogRead(s7);
  sValues[2] = analogRead(s6);
  sValues[3] = analogRead(s5);
  sValues[4] = analogRead(s4);
  sValues[5] = analogRead(s3);
  sValues[6] = analogRead(s2);
  sValues[7] = analogRead(s1);
}


int calculateError(int sValues[]) {
  int error = 0;

  // Check if center sensors (4th and 5th) are detecting the white line
  if (sValues[3] < threshold && sValues[4] < threshold) {
    error = 0;  // Both 4th and 5th sensors are on the white line (center)
  } else {
    // Check left side sensors (s1, s2, s3)
    for (int i = 0; i < 3; i++) {
      if (sValues[i] < threshold) {
        // The error is proportional to the distance from the center
        error = - (3 - i);  // Largest error for s1, smallest for s3
        break;
      }
    }
    
    // Check right side sensors (6th, 7th, 8th)
    for (int i = 7; i > 4; i--) {
      if (sValues[i] < threshold) {
        // The error is proportional to the distance from the center
        error = (i - 4);  // Largest error for s8 (i=7), smallest for s6 (i=5)
        break;
      }
    }


  }

  return error;
}

void PIDMotorControl(int error) {
  // Calculate PID terms
  float derivative = error - previousError;
  integral += error;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;

  // Calculate the PID output (adjust the formula for your application)
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Limit the output to a max value (to prevent overshooting)
  if (output > 255) output = 255;
  if (output < -255) output = -255;

  // Print the PID output for debugging
  Serial.print("PID Output: ");
  Serial.println(output);

  // Apply PID correction to the motors
  int leftMotorSpeed = motorSpeed - output;  
  int rightMotorSpeed = motorSpeed + output;

  // Ensure motor speeds are within PWM range (0-255)
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  if (leftMotorSpeed > 255) leftMotorSpeed = 255;

  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (rightMotorSpeed > 255) rightMotorSpeed = 255;

  // Set the motor speeds (same direction for both motors)
  digitalWrite(motorLeftDir, HIGH);  // Left motor forward
  analogWrite(motorLeftPWM, leftMotorSpeed);  // Set left motor speed

  digitalWrite(motorRightDir, HIGH);  // Right motor forward
  analogWrite(motorRightPWM, rightMotorSpeed);  // Set right motor speed

  // Update the previous error for the next loop
  previousError = error;
}