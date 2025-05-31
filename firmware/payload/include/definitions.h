#include <Arduino.h>
#include <Servo.h>

#define S1 PIN_F0
#define S2 PIN_F1
#define S3 PIN_F4
#define S4 PIN_F5
#define LASER_EN PIN_F6
#define SZ_SIG PIN_B7
#define TRK_SIG PIN_D0
#define CAL_BTN PIN_D1

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

bool TRK = false;
bool laser_state = false;

// Tracking binary thresholding definitions
int8_t dx = 0;
int8_t dy = 0;

int8_t norm_dy;
int8_t norm_dx;