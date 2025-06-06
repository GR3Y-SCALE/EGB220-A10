#include "PIDMotorControl.h"

#include <Arduino.h>
#include "Arduino_pin.h"

extern int motorSpeed;

// PID constants
float Kp = 15.0;  
float Ki = 0.05; 
float Kd = 0.5;  

// Variables for PID
float previousError = 0;
float integral = 0;


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
  
    // Motor control logic based on PID output
    //int motorSpeed = 50;  // Base motor speed (you can adjust this)
  
    // Apply PID correction to the motors
    int leftMotorSpeed = motorSpeed - output;  
    int rightMotorSpeed = motorSpeed + output;
  
    // Ensure motor speeds are within PWM range (0-255)
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    if (leftMotorSpeed > 255) leftMotorSpeed = 255;
  
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (rightMotorSpeed > 255) rightMotorSpeed = 255;
  
    // Set the motor speeds (same direction for both motors)
    //digitalWrite(motorLeftDir, HIGH);  // Left motor forward
    analogWrite(motorLeftPWM, leftMotorSpeed);  // Set left motor speed
  
    //digitalWrite(motorRightDir, HIGH);  // Right motor forward
    analogWrite(motorRightPWM, rightMotorSpeed);  // Set right motor speed
  
    // Update the previous error for the next loop
    previousError = error;
  }
  