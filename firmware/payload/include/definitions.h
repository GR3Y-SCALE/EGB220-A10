// #include <Arduino.h>
// #include <Servo.h>

#define S1 21
#define S2 20
#define S3 19
#define S4 18
#define LASER_EN PIN_F6
#define SZ_SIG PIN_B7
#define TRK_SIG PIN_D0
#define CAL_BTN 22
#define YELLOW PIN_B1
#define GREEN PIN_B0

// /* SENSOR CONFIGURATION
//    _________________
//   /      |  |       \
//  /  [S3] |  |  [S2]  \
// |________|  |_________|
// |________|  |_________|
// |        |  |         |
//  \  [S4] |  |  [S1]  /
//   \______|__|_______/

// */

// Servo servoX;
// Servo servoY;
// const int8_t IRSENSORS[4] = {S2,S3,S1,S4}; // Order of IR sensors on the PCB
// const int8_t numSensors = 4;

// int sensorCalibration[numSensors] = {1000,1000,1000,};
// const int detectionThreshold = 20; // 20 5mV increments, therefore 100mV threshold

// int8_t servoXRestPos = 90;
// int8_t servoYRestPos = 90;

// // Function definitions
// void calibrateSensors();
// bool readSensor(int8_t i);
// bool targetAcquired();
// void scanServoX();

// const int8_t sensor_dx[numSensors] = {-1,1,-1,1};
// const int8_t sensor_dy[numSensors] = {1,1,-1,-1};

// bool TRK = false;
// bool laser_state = false;

// // Tracking binary thresholding definitions
// int8_t dx = 0;
// int8_t dy = 0;

// int8_t norm_dy;
// int8_t norm_dx;

