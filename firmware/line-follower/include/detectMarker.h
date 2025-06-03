#include <Arduino.h>
#include <Arduino_pin.h>

//Calibrate threshold for White(StartStop and Turning), Red and Green, all threshold are the maximum value for each colour
#define RED 1000
#define GREEN 600
#define StartStop 300 // This is a white marker
#define Turning 300 // This is a white marker

extern int error;

extern bool SZ;
extern bool SSMarker;
extern bool CURVED;
extern bool WaitingToExitCurve;

void detectMarker (void);

void SZDetector(void);
void SSDetector(void);
void CURDetector (void);