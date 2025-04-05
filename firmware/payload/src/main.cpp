#include <Arduino.h>
#include <Servo.h>

#define S1 PIN_F0
#define S2 PIN_F1
#define S3 PIN_F4
#define S4 PIN_F5
#define laser_en PIN_F6
#define SZ_sig PIN_B7
#define TRK_sig PIN_D0
#define CAL_BTN PIN_D1

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

Servo servoX;
Servo servoY;
const int8_t IRSENSORS[4] = {S3,S2,S4,S1}; // Order of IR sensors on the PCB
const int8_t numSensors = 4;
double sensorCalibration[numSensors] = {0,0,0,0};
const float detectionThreshold = 0.100; // 100mV drop from background level
const float hysteresisMargin = 0.020; // 20mV hysteresis margin to stop fluctuating sensor states. 
int8_t servoXRestPos = 90;
int8_t servoYRestPos = 90;
volatile bool calibrationRequested = false;

// Function definitions
void calibrateSensors();
bool readSensor(int8_t i);
bool targetAcquired();
void scanServoX();
void onCalibrateISR();

const int8_t sensor_dx[numSensors] = {-1,1,-1,1};
const int8_t sensor_dy[numSensors] = {1,1,-1,-1};


void setup () {
    for (int8_t i = 0; i < numSensors; i++) {
        pinMode(IRSENSORS[i],INPUT);
    }
    pinMode(laser_en, OUTPUT);
    pinMode(SZ_sig, INPUT);
    servoX.attach(PIN_B5);
    servoY.attach(PIN_B6);
    pinMode(TRK_sig, OUTPUT);
    digitalWrite(TRK_sig,LOW);
    servoX.write(servoXRestPos);
    servoY.write(servoYRestPos);

    pinMode(CAL_BTN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(CAL_BTN), onCalibrateISR, FALLING);
}
void loop () {
    // TODO:
    // if (calibrationRequested) {
    //     calibrateSensors();
    //     calibrationRequested = false;
    // }
    if (!digitalRead(CAL_BTN)) {
        calibrateSensors(); // Calibrate when CAL button has been set to low, it is on internal pull up.
    }

    bool TRK = false;
    bool laser = false;

    if (digitalRead(SZ_sig)) { // If in slow zone, activate turret. This will be a 3v3 signal from the line-follower mainboard.
        if (targetAcquired()) {
            TRK = true;
            int8_t dx = 0;
            int8_t dy = 0;
            for (int8_t i = 0; i < numSensors; i++) {
                bool sensorValue = readSensor(i);
                dx += sensorValue * sensor_dx[i];
                dy += sensorValue * sensor_dy[i];
            }
            int8_t norm_dx = (dx > 0) - (dx < 0); //Normalise the difference in dx dy to 1 or 0.
            int8_t norm_dy = (dy > 0) - (dy < 0);
            if (norm_dx || norm_dy) {
                servoX.write(constrain(servoX.read() + norm_dx * 2, 0, 180));
                servoY.write(constrain(servoY.read() + norm_dy * 2, 0, 180));
            } else {
                laser = true;
            }
        } else { //Target lost or not found, scan to find target.
            TRK = false;
            laser = false;
            scanServoX();
        }

        digitalWrite(TRK_sig,TRK);
        digitalWrite(laser_en,laser);

    } else {
        TRK = false;
        laser = false;
        servoX.write(servoXRestPos);
        servoY.write(servoYRestPos);
    }
    delay(20);
}

void calibrateSensors() {
    for (int8_t i = 0; i < numSensors; i++) {
        sensorCalibration[i] = analogRead(IRSENSORS[i]) * (5.0 / 1023.0); // converts to Volts
    }
}

bool readSensor(int8_t i) {
    double rawSensorValue = analogRead(IRSENSORS[i]) * (5.0 / 1023.0);
    double delta = sensorCalibration[i] - rawSensorValue; // Sensor conduct when target is in view, dropping voltage.
    
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

    if (angle >= 180) {
        angle = 180;
        dir = false;
    } else if (angle <= 0) {
        angle = 0;
        dir = true;
    }
    servoX.write(constrain(angle, 0, 180));
}

// This is a nice to have, get working later.
// void onCalibrateISR() {
//     static unsigned long lastInterruptTime = 0;
//     unsigned long now = millis();
//     if (now - lastInterruptTime > 50) {
//         calibrationRequested = true;
//         lastInterruptTime = now;
//     }
// }