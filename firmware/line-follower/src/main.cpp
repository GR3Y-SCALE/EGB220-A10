#include <Arduino.h>

#include <Arduino_pin.h>
#include <readsensors.h>
#include <calculateError.h>
#include <motorControl.h>
#include <detectMarker.h>

//Sensors reading storage
int sValues[8];
int mValues[2];

bool SZ = false; //true when red is detected, false when green is detected
bool SSDetected = false; //true when start stop marker detected
bool SSMarker = false; //true when passed start marker, false when in startstop zone
bool CURVED = false; //True when turning, false when not.

const int sensorPins[] = {s1, s2, s3, s4, s5, s6, s7, s8};

int error = 0;
int lap = 0; 

int motorSpeed = 50;

const int motorTopSpeed = 120;

enum State {
  DEBUG,
  IDLE,
  STRAIGHT,
  TURNING,
  SLOW_ZONE,
  STOPPING,
  STOPPED
};

#ifdef DEBUGGING // Checks for debugging flag
State currentState = DEBUG;
#else
State currentState = IDLE;
#endif

void readsensors(void){
    // Read values from the IR sensors using defined pin names and store them to sValues[]
    for(int i = 0; i < 8; i++){
      sValues[i] = analogRead(sensorPins[i]);
    }
  }

void setup() {
  // Initialize IR sensor pins as INPUT
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);

  //Initialize Left Right Marker Sensors
  //pinMode(sL, INPUT);
  pinMode(sR, INPUT); // Unused

  //Initialize LEDS
  pinMode(RLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  pinMode(GLED, OUTPUT);

  //initialize push button 
  pinMode(StartButton, INPUT);
  
  // Initialize Left and Right Motor 
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);

  // Initialize Payload Pins
  pinMode(SZ_SIG, OUTPUT);
  pinMode(TRK_SIG, INPUT);

  Serial.begin(9600);  // Start serial communication for debugging
}


void loop() {
  //Read line sensor and side sensors
  readsensors();
  detectMarker();
  
  //Check states
  SZDetector(); //Change SZ state if entering or leaving SZ
  SSDetector(); //Change SS state if crossing starting or finishing line
  CURDetector(); //Change CURVED state if entering and leaving a curve


  switch (currentState){
    case DEBUG: // Test follower mainboard to payload mainboard communications, servo operation and wheel motion
      Serial.println("State: DEBUG");
      delay(1000);
      // Test Slow Zone signal
      Serial.println("TESTING SZ SIGNAL");
      for (int i = 0; i < 5; i++) {
        digitalWrite(SZ_SIG, HIGH);
        Serial.print("...ON");
        delay(500);
        digitalWrite(SZ_SIG, LOW);
        Serial.print("...OFF");
        delay(500);
      }
      Serial.println("...DONE!");
      delay(5000);

      Serial.println("TESTING TRK SIGNAL");
      pinMode(TRK_SIG,OUTPUT);
      for (int i = 0; i < 5; i++) {
        digitalWrite(TRK_SIG, HIGH);
        Serial.print("...ON");
        delay(500);
        digitalWrite(TRK_SIG, LOW);
        Serial.print("...OFF");
        delay(500);
      }
      Serial.println("...DONE!");
      digitalWrite(BUILTIN_LED, HIGH); 
      
    case IDLE:
      Serial.println("State: IDLE");
      digitalWrite(RLED,HIGH); //red led on

      if(analogRead(StartButton)){           //if button is pressed
      digitalWrite(RLED,LOW);//red led off
      motorSpeed = 50;
      currentState = STRAIGHT;
      }
      break;
    
    case STRAIGHT:
      Serial.println("State: STRAIGHT");

      digitalWrite(GLED,HIGH); //green led on
      error = calculateError(sValues);
      PIDMotorControl(error);
      delay(1);

      motorSpeed += 5;
      if (motorSpeed > motorTopSpeed) motorSpeed = motorTopSpeed;

      if(SZ){
        digitalWrite(GLED,LOW);//turn green led off
        currentState = SLOW_ZONE;
      }

      if(SSDetected && !SSMarker){
        digitalWrite(GLED,LOW);//turn green led off
        currentState = STOPPING;
      }

      if(CURVED){
        digitalWrite(GLED,LOW);//turn green led off
        currentState = TURNING;
      }
      break;

    case TURNING:
      Serial.println("State: TURNING");

      motorSpeed = 50;
      error = calculateError(sValues);
      PIDMotorControl(error);

      if(!CURVED){
        currentState = STRAIGHT;
      }
      break;

    case SLOW_ZONE:
      Serial.println("State: SLOW_ZONE");
      
      digitalWrite(SZ_SIG, HIGH); //send high to payload pin

      motorSpeed = 25;
      error = calculateError(sValues);
      PIDMotorControl(error);  

      digitalWrite(GLED,HIGH);//turn on green led for light following
      
      //if log bumper switch is high
      digitalWrite(BLED,HIGH);//if in contact with obs, turn blue led on
      // else
      digitalWrite(BLED, LOW);

      if(digitalRead(TRK_SIG)){
        //digitalWrite(YLED,HIGH); Yellow LED on when tracking
      }
      else{
        //digitalWrite(YLED,LOW);
      }

      if(!SZ){
        digitalWrite(SZ_SIG, LOW);  //send low to payload pin
        digitalWrite(GLED,LOW);
        currentState = STRAIGHT;
        motorSpeed = 50;
      }
      break;

    case STOPPING:
      Serial.print("State: STOPPING");

      digitalWrite(RLED,HIGH);//turn red led on

      motorSpeed -= 5;
      if (motorSpeed<0){
        motorSpeed = 0; 
        currentState = STOPPED;
      }

      error = calculateError(sValues);
      PIDMotorControl(error);
      break;

    case STOPPED:
     Serial.print("State: STOPPED");

     stopMotors();

     delay (2500);

     if(lap<3){
      digitalWrite(RLED,LOW);//red led off
      currentState = STRAIGHT;
      motorSpeed = 50;
      error = 0;
     }
     else{
      currentState = IDLE;
      lap = 0;
     }

     break;

  }
}


