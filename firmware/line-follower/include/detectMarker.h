#include <Arduino.h>
#include <Arduino_pin.h>

//Calibrate threshold for White(StartStop and Turning), Red and Green, all threshold are the maximum value for each colour
#define Red 1000
#define Green 600
#define StartStop 300
#define Turning 300

extern int error;

extern bool SZ;
extern bool SSMarker;
extern bool CURVED;
extern bool WaitingToExitCurve;

void detectMarker (void);

void SZDetector(void);
void SSDetector(void);
void CURDetector (void);