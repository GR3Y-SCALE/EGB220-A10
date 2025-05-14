// Define sensor pins(from Right to Left) using #define
#define s1 A7
#define s2 A6
#define s3 A5
#define s4 A4
#define s5 A3
#define s6 A2
#define s7 A1
#define s8 A0

//Define Left Right Marker Sensor
#define sL A4   //reconfig pin during tuning
#define sR A5   //reconfig pin during tuning

//Define Red, Green, Blue LED
#define RLED 0
#define GLED 1
#define BLED 2

// Motor Pins
//#define motorLeftDir 7      // Direction pin for left motor
#define motorLeftPWM 9     // PWM pin for left motor speed
//#define motorRightDir 17     // Direction pin for right motor
#define motorRightPWM 4    // PWM pin for right motor speed

//Define Push Button
#define StartButton 22

//Define Payload System Pin
#define SZ_SIG 13
#define TRK_SIG 12