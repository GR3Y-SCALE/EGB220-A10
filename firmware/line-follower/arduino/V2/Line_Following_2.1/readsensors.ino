#include "readsensors.h"

#include <Arduino.h>
#include "Arduino_pin.h"

extern int sValues[];
const int sensorPins[] = {s8, s7, s6, s5, s4, s3, s2, s1};

void readsensors(void){
    // Read values from the IR sensors using defined pin names and store them to sValues[]
    for(int i = 0; i < 8; i++){
      sValues[i] = analogRead(sensorPins[i]);
    }
  }