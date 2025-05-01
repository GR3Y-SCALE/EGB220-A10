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

<<<<<<< Updated upstream
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

=======
/* PORTAL-INSPIRED TRACKING TURRET WITH IR SENSOR ARRAY
 *                                      
 *              ╭──────────────────╮                 
 *              │  ╭────────────╮  │    "Are you still there?"
 *              │  │ APERTURE   │  │      
 *              │  │ SCIENCE    │  │    "I see you!"    
 *              │  ╰────────────╯  │      
 *              │       ╭──╮       │    "Deploying!"
 *         ╭────┴───────┤••├───────┴────╮
 *         │    SENSORS  \/  ACTIVATED  │
 *         │                            │
 *         │  ╭────╮         ╭────╮    │  ╔═ TARGET ACQUISITION UNIT ═╗
 *         │  │ S3 │         │ S2 │    │  ║                          ║
 *         │  ╰────╯         ╰────╯    │  ║  [S3]          [S2]      ║
 *         │           (●)             │  ║   │              │       ║
 *         │         APERTURE          │  ║   │      ⚡      │       ║
 *         │          TURRET           │  ║   │              │       ║
 *         │  ╭────╮         ╭────╮    │  ║   │              │       ║
 *         │  │ S4 │         │ S1 │    │  ║  [S4]          [S1]      ║
 *         │  ╰────╯         ╰────╯    │  ╚══════════════════════════╝
 *         │                            │
 *         ╰────┬─────────────────┬────╯
 *              │                 │
 *              ╰────╮      ╭────╯
 *                   │      │
 *                  /        \
 *                 /   LEGS   \
 *                /_          _\
 *    
 *    "I don't hate you"     "I'm different"
 */
>>>>>>> Stashed changes
Servo servoX;
Servo servoY;
const int8_t IRSENSORS[4] = {S2,S3,S1,S4}; // Order of IR sensors on the PCB
const int8_t numSensors = 4;

<<<<<<< Updated upstream
int sensorCalibration[numSensors] = {0,0,0,0};
const int detectionThreshold = 20; // 20 5mV increments, therefore 100mV threshold
=======
int sensorCalibration[numSensors] = {1033,1033,1033,1033};
const int detectionThreshold = 1; // 20 5mV increments, therefore 100mV threshold
const int minSensorDelta = 5; // Minimum delta to consider for differential tracking
>>>>>>> Stashed changes

int8_t servoXRestPos = 90;
int8_t servoYRestPos = 90;

/*
 * =============================================================================================
 * LAGRANGIAN-BASED KALMAN FILTER IMPLEMENTATION
 * =============================================================================================
 *
 *    ┌───────────────┐         ┌───────────────┐         ┌───────────────┐
 *    │  PREDICTION   │         │ MEASUREMENT   │         │    UPDATE     │
 *    │  STEP         │────────▶│               │────────▶│               │
 *    │ (Lagrangian)  │         │ (IR Sensors)  │         │ (Combine)     │
 *    └───────────────┘         └───────────────┘         └───────────────┘
 *           │                                                    │
 *           │                                                    │
 *           └────────────────────────────────────────────────────┘
 *                               (Loop)
 *
 * Lagrangian Mechanics: L = T - U, where:
 *   T = Kinetic Energy = (1/2)m(vx² + vy²)
 *   U = Potential Energy = 0 (flat space)
 * 
 * Euler-Lagrange equation: d/dt(∂L/∂q̇) - ∂L/∂q = 0
 * Gives us: d/dt(m·v) - 0 = 0, therefore m·a = 0 and a = 0
 * This confirms our constant velocity model is optimal in this case.
 */

// Kalman Filter variables with Lagrangian interpretations
// State vector: [posX, velX, posY, velY]
float X[4] = {0, 0, 0, 0};              // Generalized coordinates (q, q̇)
float P[4][4] = {{1, 0, 0, 0},          // Covariance of the coordinates
                 {0, 1, 0, 0},          // Higher values = more uncertain
                 {0, 0, 1, 0},
                 {0, 0, 0, 1}};
// Energy dissipation factors (non-conservative forces)
float Q[4] = {0.01, 0.02, 0.01, 0.02};  // Process noise (uncertainty growth rate)
                                        // Higher Q = trust measurements more
float R = 0.1;                          // Measurement noise (sensor uncertainty)
                                        // Higher R = trust predictions more
float dt = 0.06;                        // Time step (matches loop delay)
unsigned long lastKalmanTime = 0;

// Function definitions
void calibrateSensors();
int readSensorDelta(int8_t i);
bool targetAcquired();
void scanServoX();
void lagrangianKalmanPredict();
void kalmanUpdate(float measX, float measY);
void applyKalmanOutput();

const int8_t sensor_dx[numSensors] = {-1,1,-1,1};
const int8_t sensor_dy[numSensors] = {1,1,-1,-1};


void setup () {
    for (int8_t i = 0; i < numSensors; i++) {
        pinMode(IRSENSORS[i],INPUT);
    }
    pinMode(LASER_EN, OUTPUT);
    pinMode(SZ_SIG, INPUT);
    servoX.attach(PIN_B5);
    servoY.attach(PIN_B6);
    pinMode(TRK_SIG, OUTPUT);
    digitalWrite(TRK_SIG,LOW);
    servoX.write(servoXRestPos);
    servoY.write(servoYRestPos);

    pinMode(CAL_BTN, INPUT_PULLUP);
    lastKalmanTime = millis();
    
    // Initialize Kalman filter state to servo rest positions
    X[0] = 0;  // X position (normalized -1 to 1)
    X[2] = 0;  // Y position (normalized -1 to 1)
}

void loop () {
    // Calculate dt (time since last loop)
    unsigned long currentTime = millis();
    dt = (currentTime - lastKalmanTime) / 1000.0;
    lastKalmanTime = currentTime;
    
    // Limit dt to avoid large jumps
    if (dt > 0.2) dt = 0.2;
    
    if (!digitalRead(CAL_BTN)) {
        calibrateSensors(); // Calibrate when CAL button has been set to low, it is on internal pull up.
        // Reset Kalman filter
        X[0] = 0; X[1] = 0; X[2] = 0; X[3] = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    bool TRK = false;
    bool laser = false;

    if (digitalRead(SZ_SIG)) { // If in slow zone, activate turret. This will be a 3v3 signal from the line-follower mainboard.
        if (targetAcquired()) {
            TRK = true;
            int16_t dx = 0;
            int16_t dy = 0;
            int16_t totalSignalStrength = 0;
            
            for (int8_t i = 0; i < numSensors; i++) {
<<<<<<< Updated upstream
                bool sensorValue = readSensor(i);
                dx += sensorValue * sensor_dx[i];
                dy += sensorValue * sensor_dy[i];
=======
                int sensorDelta = readSensorDelta(i);
                if (sensorDelta > minSensorDelta) {
                    dx += sensorDelta * sensor_dx[i];
                    dy += sensorDelta * sensor_dy[i];
                    totalSignalStrength += sensorDelta;
                }
>>>>>>> Stashed changes
            }
            
            // Process measurements with Kalman filter
            if (totalSignalStrength > 0) {
                // Calculate normalized measurements (-1.0 to 1.0)
                float measX = (float)dx / totalSignalStrength;
                float measY = (float)dy / totalSignalStrength;
                
                // ===== LAGRANGIAN KALMAN FILTER PIPELINE =====
                //  1. Predict where target should be now (based on Lagrangian mechanics)
                lagrangianKalmanPredict();
                
                //  2. Update our belief with new sensor measurements
                kalmanUpdate(measX, measY);
                
                //  3. Use our best estimate to control servos
                applyKalmanOutput();
                
                // If position estimate is very close to center and velocity is low, enable laser
                if (fabs(X[0]) < 0.08 && fabs(X[2]) < 0.08 && 
                    fabs(X[1]) < 0.05 && fabs(X[3]) < 0.05) {
                    laser = true;
                }
            }
        } else { //Target lost or not found, scan to find target.
            TRK = false;
            laser = false;
<<<<<<< Updated upstream
            scanServoX();
=======
            // Reset velocity estimates when target is lost
            X[1] = 0;
            X[3] = 0;
            scanServoX(); // Enable scanning to find target
>>>>>>> Stashed changes
        }

        digitalWrite(TRK_SIG,TRK);
        digitalWrite(LASER_EN,laser);

    } else {
        TRK = false;
        laser = false;
        servoX.write(servoXRestPos);
        servoY.write(servoYRestPos);
        // Reset Kalman filter state when not in slow zone
        X[0] = 0; X[1] = 0; X[2] = 0; X[3] = 0;
    }
    delay(20);
}

/*
 * =============================================================================================
 * STEP 1: LAGRANGIAN PREDICTION - "How does the system evolve according to its energy?"
 * =============================================================================================
 *
 *    Lagrangian L(q,q̇) = T(q̇) - U(q)
 *    
 *    Euler-Lagrange Equations:          Constant Velocity Solution:
 *    ┌───────────────────────┐          ┌───────────────┐
 *    │ d  ∂L     ∂L          │          │ q(t+dt) = q(t) + q̇(t)·dt │
 *    │ -- ─── - ─── = 0      │ ────────▶│               │
 *    │ dt ∂q̇     ∂q           │          │ q̇(t+dt) = q̇(t)│
 *    └───────────────────────┘          └───────────────┘
 *         
 *    The Lagrangian approach derives our motion model from energy principles.
 *    In this flat space without potential gradients, objects maintain constant velocity.
 */
void lagrangianKalmanPredict() {
    // Lagrangian mechanics with constant velocity model
    // From Euler-Lagrange equations: d/dt(∂L/∂q̇) = ∂L/∂q
    // In a conservative system with no external forces, this gives us:
    // q(t+dt) = q(t) + q̇(t)·dt
    // q̇(t+dt) = q̇(t)
    
    // Update positions based on velocities (generalized coordinates evolve with time)
    X[0] += X[1] * dt;  // x = x + vx·dt
    X[2] += X[3] * dt;  // y = y + vy·dt
    
    // Update covariance matrix P using the Lagrangian propagation model
    // P = F·P·F' + Q, where F is derived from the Euler-Lagrange equations
    float F[4][4] = {{1, dt, 0, 0},   // This matrix represents our Lagrangian dynamics
                     {0, 1, 0, 0},     // Where position evolves with velocity
                     {0, 0, 1, dt},    // And velocity is constant (no acceleration)
                     {0, 0, 0, 1}};
    
    // Matrix multiplication: F·P
    float FP[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    
    // Matrix multiplication: F·P·F'
    float FPFt[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FPFt[i][j] += FP[i][k] * F[j][k]; // Note: F' = transpose of F
            }
        }
    }
    
    // Add process noise Q to account for non-conservative forces
    // These represent energy dissipation and random forces not in the Lagrangian
    for (int i = 0; i < 4; i++) {
        FPFt[i][i] += Q[i];
    }
    
    // Update P with new uncertainty
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = FPFt[i][j];
        }
    }
}

/*
 * =============================================================================================
 * STEP 2: UPDATE - "Blend predictions with measurements optimally"
 * =============================================================================================
 *
 *    ┌───────────┐     ┌───────────┐     ┌───────────┐     ┌───────────┐
 *    │ Predicted │     │  Sensor   │     │  Kalman   │     │  Updated  │
 *    │  State    │────▶│ Difference│────▶│   Gain    │────▶│   State   │
 *    │           │     │           │     │           │     │           │
 *    └───────────┘     └───────────┘     └───────────┘     └───────────┘
 *                                              │
 *                        How much to trust     │
 *                      predictions vs sensors  ▼
 *
 *    Kalman Gain K:  Higher P → Trust measurements more
 *                    Higher R → Trust predictions more 
 */
void kalmanUpdate(float measX, float measY) {
    // Measurement model: z = H*x + v, where H maps state to measurements
    // For position-only measurements, H = [1 0 0 0; 0 0 1 0]
    // This means we only directly measure position, not velocity
    
    // Calculate Kalman gain
    // K = P*H'/(H*P*H' + R)
    // Kalman gain determines how much to trust measurements vs predictions
    float K[4][2] = {{0}}; // Kalman gain
    
    // For position-only measurements:
    // K[0][0] = P[0][0] / (P[0][0] + R)   <-- X position update factor
    // K[1][0] = P[1][0] / (P[0][0] + R)   <-- X velocity update factor
    // K[2][1] = P[2][2] / (P[2][2] + R)   <-- Y position update factor
    // K[3][1] = P[3][2] / (P[2][2] + R)   <-- Y velocity update factor
    
    K[0][0] = P[0][0] / (P[0][0] + R);
    K[1][0] = P[1][0] / (P[0][0] + R);
    K[2][1] = P[2][2] / (P[2][2] + R);
    K[3][1] = P[3][2] / (P[2][2] + R);
    
    // Update state estimate with measurement
    // X = X + K * (z - H*X)
    // (z - H*X) is the "innovation" - the difference between measured and predicted
    float innovation_x = measX - X[0];  // How wrong was our position prediction?
    float innovation_y = measY - X[2];
    
    // Apply corrections weighted by Kalman gain
    X[0] += K[0][0] * innovation_x;  // Update position directly
    X[1] += K[1][0] * innovation_x;  // MAGIC: Also updates velocity based on position error!
    X[2] += K[2][1] * innovation_y;
    X[3] += K[3][1] * innovation_y;
    
    // Update covariance matrix (reduce uncertainty based on measurements)
    // P = (I - K*H) * P
    float KH[4][4] = {{0}};
    KH[0][0] = K[0][0];
    KH[1][0] = K[1][0];
    KH[2][2] = K[2][1];
    KH[3][2] = K[3][1];
    
    // Calculate (I - K*H) - Identity matrix minus KH
    float IKH[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            IKH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }
    
    // Calculate (I - K*H) * P - update uncertainty
    float IKHP[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                IKHP[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }
    
    // Update P with reduced uncertainty (measurements reduce uncertainty)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = IKHP[i][j];
        }
    }
    
    // Constrain state to reasonable values
    X[0] = constrain(X[0], -1.0, 1.0);   // X position
    X[1] = constrain(X[1], -2.0, 2.0);   // X velocity
    X[2] = constrain(X[2], -1.0, 1.0);   // Y position
    X[3] = constrain(X[3], -2.0, 2.0);   // Y velocity
}

/*
 * =============================================================================================
 * STEP 3: CONTROL - "Use our best state estimate to move servos smoothly"
 * =============================================================================================
 *
 *    Target State              Control Law                 Servo
 *    ┌───────────┐            ┌───────────┐             ┌───────┐
 *    │ Position  │            │ Position  │             │       │
 *    │ Velocity  │───────────▶│    +      │────────────▶│  PWM  │
 *    │           │            │ Velocity  │             │       │
 *    └───────────┘            └───────────┘             └───────┘
 *                          (Predictive control)
 */
void applyKalmanOutput() {
    // Calculate servo movements based on position and velocity estimates
    // This is a "predictive controller" because it uses velocity to anticipate movement
    
    // Position component: -X[0] * 20.0  (negative feedback to center target)
    // Predictive component: -X[1] * 5.0  (counteract velocity)
    float moveX = -X[0] * 20.0 - X[1] * 5.0; // Proportional + predictive component
    float moveY = -X[2] * 20.0 - X[3] * 5.0;
    
    // Scale and constrain servo movements
    int8_t servoMoveX = constrain((int8_t)moveX, -3, 3);
    int8_t servoMoveY = constrain((int8_t)moveY, -3, 3);
    
    // Apply to servos
    servoX.write(constrain(servoX.read() + servoMoveX, 0, 180));
    servoY.write(constrain(servoY.read() + servoMoveY, 0, 180));
}

void calibrateSensors() {
    for (int8_t i = 0; i < numSensors; i++) {
        sensorCalibration[i] = analogRead(IRSENSORS[i]); // converts to Volts
    }
}

// Returns the delta from calibration (0 if below threshold)
int readSensorDelta(int8_t i) {
    int rawSensorValue = analogRead(IRSENSORS[i]);
<<<<<<< Updated upstream
    int delta = round(sensorCalibration[i] - rawSensorValue); // Sensor conduct when target is in view, dropping voltage.
=======
    int delta = sensorCalibration[i] - rawSensorValue; // Sensor conduct when target is in view, dropping voltage.
>>>>>>> Stashed changes
    
    return (delta > detectionThreshold) ? delta : 0; // Return actual delta if above threshold, otherwise 0
}

bool targetAcquired() { // Check if any sensors see the target.
    for (int8_t i = 0; i < numSensors; i++) {
        if (readSensorDelta(i) > detectionThreshold) return true;
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
