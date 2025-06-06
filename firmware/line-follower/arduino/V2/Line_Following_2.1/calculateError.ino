#include "calculateError.h"

#include <Arduino.h>

extern int lap;
extern bool SZ;

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