#include <MotorControl.h>

#include <Arduino.h>
#include <Arduino_pin.h>

extern int motorSpeed;

extern int lap;
extern bool SZ;

// PID constants
float Kp = 15.0;  
float Ki = 0.05; 
float Kd = 0.5;  

float previousError = 0;
float integral = 0;

const int baseMotorSpeed = 120;


void PIDMotorControl(int error) {
  float derivative = error - previousError;
  integral += error;

  // Calculate the PID output
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Limit the output to a max value
  if (output > 255) output = 255;
  if (output < -255) output = -255;

  Serial.print("PID Output: ");
  Serial.println(output);

  // Apply PID correction to the motors
  int leftMotorSpeed = baseMotorSpeed - output;  
  int rightMotorSpeed = baseMotorSpeed + output;

  // Ensure motor speeds are within PWM range (0-255)
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  if (leftMotorSpeed > 255) leftMotorSpeed = 255;

  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (rightMotorSpeed > 255) rightMotorSpeed = 255;

  digitalWrite(motorLeftB, LOW);
  analogWrite(motorLeftA, leftMotorSpeed);

  digitalWrite(motorRightB, LOW);
  analogWrite(motorRightA, rightMotorSpeed);

  // Update the previous error for the next loop
  previousError = error;
}

void stopMotors() {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, LOW);
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightA, LOW);
}

int calculateError(int sValues[]) {
    int error = 0;
    int threshold = 300;  // Adjust threshold based on sensor calibration (<300 is white line)

    //if in slow zone and left half and right half of sensors is on the track
    if(SZ && (sValues[0]<threshold||sValues[1]<threshold||sValues[2]<threshold||sValues[3]<threshold) && (sValues[4]<threshold||sValues[5]<threshold||sValues[6]<threshold||sValues[7]<threshold)){
      //if any 3 sensors on the left is on track and lap is odd number
      if(sValues[0]<threshold||sValues[1]<threshold||sValues[2]<threshold && (lap % 2)){
        for (int i = 0; i < 3; i++) {
          if (sValues[i] < threshold) {
             // The error is proportional to the distance from the center
             error = - (3 - i);  // Largest error for s1, smallest for s3
             break;
           }
        }
      }
      //if any 3 sensors on the right is on track and lap is even number
      else if (sValues[5]<threshold||sValues[6]<threshold||sValues[7]<threshold && !(lap % 2)){
        for (int i = 7; i > 4; i--) {
          if (sValues[i] < threshold) {
           // The error is proportional to the distance from the center
            error = (i - 4);  // Largest error for s8 (i=7), smallest for s6 (i=5)
            break;
          }
        }
      }
      else{
        error = 0;
      }
    }
    
    else{
      // Check if center sensors (4th and 5th) are detecting the white line
      if (sValues[3] < threshold && sValues[4] < threshold) {
        error = 0;  // Both 4th and 5th sensors are on the white line (center)
      } 
      else{
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
    }
    
  
    return error;
}
  