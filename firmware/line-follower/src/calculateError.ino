#include <calculateError.h>

#include <Arduino.h>

extern int lap;

int calculateError(int sValues[]) {
    int error = 0;
    int threshold = 300;  // Adjust threshold based on sensor calibration (<300 is white line)
  
    // Check if center sensors (4th and 5th) are detecting the white line
    if (sValues[3] < threshold && sValues[4] < threshold && lap) {
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