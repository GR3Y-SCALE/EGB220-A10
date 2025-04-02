#include <Arduino.h>
#include <Arduino_pin.h>

#include <readsensors.h>
#include <calculateError.h>
#include <PIDMotorControl.h>

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
  // Obtain sensor reading
  int sValues[8];  // Create an array to store the sensor readings
  readsensors(sValues);  // Pass the array to the readsensors function

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

  delay(50);  // Short delay for smooth motor control
}


