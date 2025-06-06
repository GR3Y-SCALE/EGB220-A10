// PID constants
extern float Kp;   
extern float Ki; 
extern float Kd;   

// Variables for PID
extern float previousError;
extern float integral; 

void PIDMotorControl();

void stopMotors();

extern int lap;
extern bool SZ;