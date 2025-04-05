#include <Arduino.h>
#include <Servo.h>

#define S1 PIN_A0
#define S2 PIN_A1
#define S3 PIN_A2
#define S4 PIN_A3
#define LASER_EN PIN0
#define SZ_SIG 1
#define TRK_SIG 2
#define CAL_BTN 8

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
const int8_t IRSENSORS[4] = {S2,S3,S1,S4}; // Order of IR sensors on the PCB
const int8_t numSensors = 4;

int sensorCalibration[numSensors] = {0,0,0,0};
const int detectionThreshold = 20; // 20 5mV increments, therefore 100mV threshold

int8_t servoXRestPos = 90;
int8_t servoYRestPos = 90;

// Function definitions
void calibrateSensors();
bool readSensor(int8_t i);
bool targetAcquired();
void scanServoX();

const int8_t sensor_dx[numSensors] = {-1,1,-1,1};
const int8_t sensor_dy[numSensors] = {1,1,-1,-1};


void setup () {
    for (int8_t i = 0; i < numSensors; i++) {
        pinMode(IRSENSORS[i],INPUT);
    }
    pinMode(LASER_EN, OUTPUT);
    pinMode(SZ_SIG, INPUT);
    servoX.attach(9);
    servoY.attach(10);
    pinMode(TRK_SIG, OUTPUT);
    digitalWrite(TRK_SIG,LOW);
    servoX.write(servoXRestPos);
    servoY.write(servoYRestPos);

    pinMode(CAL_BTN, INPUT_PULLUP);
}
void loop () {
    if (!digitalRead(CAL_BTN)) {
        calibrateSensors(); // Calibrate when CAL button has been set to low, it is on internal pull up.
    }

    bool TRK = false;
    bool laser = false;

    if (digitalRead(SZ_SIG)) { // If in slow zone, activate turret. This will be a 3v3 signal from the line-follower mainboard.
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

        digitalWrite(TRK_SIG,TRK);
        digitalWrite(LASER_EN,laser);

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
