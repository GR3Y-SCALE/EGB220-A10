#include <Arduino.h>
#include <Arduino_pin.h>
#include <detectMarker.h>

extern int mValues[];
extern int lap;

extern bool SSDetected; 
bool CURDetected = false; 

bool WaitingToExitCurve = false;
unsigned long timer = 0;

//For threshold Calibration Use
// const int LRSensorPin = 0;

// void detectMarker (void){
//     for(int i = 0; i < 2; i++){
//         mValues[i] = analogRead(0);
//     }
// }
void detectMarker() {
    mValues[0] = analogRead(0);
} 

//Detect Slow Zone Marker
void SZDetector(void){ //Using right sensor
    if(mValues[1]>GREEN && mValues[1]<RED){
        SZ = true;
    }
    else if(mValues[1]<GREEN && mValues[1]>StartStop){
        SZ = false;
    }
}

//Detect Start Stop Marker
void SSDetector(void){
    if(mValues[1]<StartStop){
        if (!SSDetected){
            SSDetected = true;

            if(!SSMarker){
                SSMarker = true;
                lap += 1;
            }
            else{
                SSMarker = false;
            }
        }
        
    }
    else{
        SSDetected = false;
    }
}

//Detect Curved line
void CURDetector (void){
    if(mValues[0]<Turning){ // Left sensor

        if (!CURDetected){
            CURDetected = true;

            if(!CURVED){
                CURVED = true;
                WaitingToExitCurve = false;
            }
            else{
                timer = millis(); 
                WaitingToExitCurve = true; 
            }
        }
    }

    else if (CURVED && WaitingToExitCurve && (millis() - timer > 400) && (error == 0)) {
        CURVED = false;
        WaitingToExitCurve = false;
        CURDetected = false;
    }

    else if (CURVED && WaitingToExitCurve && (millis() - timer > 400) && !(error == 0)){
        CURVED = true;
        WaitingToExitCurve = false;
        CURDetected = false;
    }
    
    else{
        CURDetected = false;
    }
}