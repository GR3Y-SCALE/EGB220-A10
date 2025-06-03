#include <Arduino.h>
#include <Servo.h>
#include "definitions.h"

/* SENSOR CONFIGURATION
   _________________
  /      |  |       \
 /  [S3] |  |  [S2]  \
|________|  |_________|
|________|  |_________|
|        |  |         |
 \  [S4] |  |  [S1]  /
  \______|__|_______/

*/

enum State {
    DEBUG,
    IDLE,
    SEARCHING,
    TRACKING
};

#ifdef DEBUGGING // Checks for debugging flag
State currentState = DEBUG;
#else
State currentState = IDLE;
#endif

void setup () {
    for (int8_t i = 0; i < numSensors; i++) {
        pinMode(IRSENSORS[i],INPUT);
    }
    pinMode(LASER_EN, OUTPUT);
    pinMode(SZ_SIG, INPUT);
    servoX.attach(PIN_B5);
    servoY.attach(PIN_B6);
    pinMode(TRK_SIG, OUTPUT);
    digitalWrite(TRK_SIG,LOW);
    servoX.write(servoXRestPos);
    servoY.write(servoYRestPos);

    pinMode(CAL_BTN, INPUT_PULLUP);
}
void loop () {

    switch (currentState) {
        case DEBUG:
        pinMode(TRK_SIG, INPUT);
        while(true) {
            //Test slow zone signal
            digitalWrite(11, digitalRead(SZ_SIG) || digitalRead(TRK_SIG));
            delay(5);
        }
        break;

        case IDLE:
            if (!digitalRead(CAL_BTN)) {
                calibrateSensors(); // Calibrate when CAL button has been set to low, it is on internal pull up.
            }

            if (digitalRead(SZ_SIG)) {
                currentState = SEARCHING;
            }
            servoX.write(servoXRestPos);
            servoY.write(servoYRestPos);
            break;

        case SEARCHING:
            if (targetAcquired()) {
                currentState = TRACKING;
            } else {
                scanServoX();
            }
            break;
        
        case TRACKING:
            dx = 0;
            dy = 0;
            for (int8_t i = 0; i < numSensors; i++) {
                bool sensorValue = readSensor(i);
                dx += sensorValue * sensor_dx[i];
                dy += sensorValue * sensor_dy[i];
            }
            norm_dx = (dx > 0) - (dx < 0); //Normalise the difference in dx dy to 1 or 0.
            norm_dy = (dy > 0) - (dy < 0);
            if (norm_dx || norm_dy) {
                servoX.write(constrain(servoX.read() + norm_dx * 2, 0, 180));
                servoY.write(constrain(servoY.read() + norm_dy * 2, 0, 180));
                laser_state = false;
            } else {
                laser_state = true;
            }

            if (targetAcquired()) {
                digitalWrite(TRK_SIG, HIGH);
                digitalWrite(LASER_EN, laser_state);
            } else {
                currentState = SEARCHING;
            }
            delay(5);
            break;

        default:
            currentState = IDLE;
    break;
            
            

    }
}

void calibrateSensors() {
    for (int8_t i = 0; i < numSensors; i++) {
        sensorCalibration[i] = analogRead(IRSENSORS[i]); // converts to Volts
    }
}

bool readSensor(int8_t i) {
    int rawSensorValue = analogRead(IRSENSORS[i]);
    int delta = round(sensorCalibration[i] - rawSensorValue); // Sensor conduct when target is in view, dropping voltage.
    
    return delta > detectionThreshold; // Threshold to differentiate between background noise and target.
}

bool targetAcquired() { // Check if any sensors see the target.
    for (int8_t i = 0; i < numSensors; i++) {
        if (readSensor(i)) return true;
    }
    return false;
}

void scanServoX() {
    static int8_t angle = 0;
    static bool dir = 1;
    angle = (servoX.read() + (dir ? 5 : -5));
    if (angle >= 180) {
        angle = 180;
        dir = false;
    } else if (angle <= 0) {
        angle = 0;
        dir = true;
    }
    servoX.write(constrain(angle, 0, 180));
}