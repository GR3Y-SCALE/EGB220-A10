#include <readsensors.h>

#include <Arduino.h>
#include <Arduino_pin.h>

const int sensorPins[] = {A3, A2, A1, A0, A8, A7, A11, A6};

void readsensors(int sValues[]){
    // Read values from the IR sensors using defined pin names and store them to sValues[]
    for(int i = 0; i < 8; i++){
      sValues[i] = analogRead(sensorPins[i]);
    }
  }