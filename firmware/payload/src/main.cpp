#include <Arduino.h>
#include <Servo.h>

#define S1 PIN_F0
#define S2 PIN_F1
#define S3 PIN_F4
#define S4 PIN_F5
const int8_t IRSENSORS[4] = {S3,S2,S4,S1}; // Order of IR sensors on the PCB
const int8_t numSensors = 4;
#define laser_en PIN_F6
Servo servoX;
Servo servoY;
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

double sensorCalibration[numSensors] = {0,0,0,0};
const float detectionThreshold = 0.100; // 100mV drop from background level
int8_t servoXRestPos = 90;
int8_t servoYRestPos = 90;
volatile bool calibrationRequested = false;

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
    attachInterrupt(digitalPinToInterrupt(CAL_BTN), onCalibrateISR, FALLING);
}
void loop () {
    if (calibrationRequested) {
        calibrateSensors();
        calibrationRequested = false;
    }
    bool SZ = digitalRead(SZ_sig); // Read SlowZone Signal pin from Line Follower
    bool TRK = false;
    bool laser = false;

    if (SZ) { // If in slow zone, activate turret
        if (targetAcquired()) {
            TRK = true;
            int8_t dx = 0;
            int8_t dy = 0;
            for (int8_t i = 0; i < numSensors; i++) {
                dx += readSensor(i) * sensor_dx[i];
                dy += readSensor(i) * sensor_dy[i];
            }
            int8_t norm_dx = (dx > 0) - (dx < 0);
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

        delay(20);
    } else {
        TRK = false;
        laser = false;
        servoX.write(servoXRestPos);
        servoY.write(servoYRestPos);
    }
}

void calibrateSensors() {
    for (int8_t i = 0; i < numSensors; i++) {
        sensorCalibration[i] = analogRead(IRSENSORS[i]) * (5.0 / 1023.0); // converts to V
    }
}

bool readSensor(int8_t i) {
    double rawSensorValue = analogRead(IRSENSORS[i]) * (5.0 / 1023.0);
    return (rawSensorValue - sensorCalibration[i] > detectionThreshold);
}

bool targetAcquired() {
    bool tracking = false;
    for (int8_t i = 0; i < numSensors; i++) {
        if (readSensor(i)) tracking = true;
    }
    return tracking;
}

void scanServoX() {
    static int8_t angle = 0;
    static bool dir = 1;

    angle = (servoX.read() + dir * 5);
    if (angle >= 180 || angle <= 0) {
        dir = !dir;
    }
    servoX.write(constrain(angle, 0, 180));
}

void onCalibrateISR() {
    static unsigned long lastInterruptTime = 0;
    unsigned long now = millis();
    if (now - lastInterruptTime > 50) {
        calibrationRequested = true;
        lastInterruptTime = now;
    }
}