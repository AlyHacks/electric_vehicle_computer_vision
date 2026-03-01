//TODO calibrate Wheel base
//TODO calibrate CPR
//TODO calibrate rotation radius

#include <Wire.h>        // I2C communication library for MPU6050
#include <MPU6050.h>     // MPU6050 sensor driver

MPU6050 mpu;

/* Robot geometry and hardware parameters (physical constants) */

// Wheel diameter in meters
// 3 inches = 0.0762 meters
const float WHEEL_DIAMETER = 0.0762;

// Wheel radius in meters
const float WHEEL_RADIUS = 0.0381;

// Effective wheel track width (wheel-to-wheel distance) in meters
// TODO This value MUST be calibrated experimentally
const float WHEEL_BASE = 0.24;

// TODO Encoder resolution: counts per full wheel revolution
const int CPR = 360;


/* =========================================================
   Encoder pin assignments
   ========================================================= */


// Must be an interrupt-capable digital pin
const int ENCODER_L_A = 2;

// Right wheel encoder channel A pin
const int ENCODER_R_A = 3;


/* =========================================================
   Encoder counters (used inside interrupts)
   ========================================================= */

// Accumulated pulse count for the left encoder
// volatile ensures correctness when modified in interrupts
volatile long countL = 0;

// Accumulated pulse count for the right encoder
volatile long countR = 0;


/* =========================================================
   Robot pose state variables (world frame)
   ========================================================= */

// Current x position in meters
// Initial position is assumed to be (0, 0)
float x = 0;

// Current y position in meters
float y = 0;

// Current yaw angle in radians
// Convention: counterclockwise rotation is positive
float yaw = 0;


/* =========================================================
   Time management
   ========================================================= */

// Timestamp of the previous control loop iteration (milliseconds)
unsigned long lastTime;


/* =========================================================
   IMU-related variables
   ========================================================= */

// Gyroscope Z-axis bias (raw ADC units)
// Used to compensate for static gyro drift
float gyroZ_offset = 0;


/* =========================================================
   Encoder interrupt service routines (ISR)
   ========================================================= */

// Left encoder interrupt handler
// Increments the pulse count on each rising edge
void ISR_left() {
  countL++;
}

// Right encoder interrupt handler
void ISR_right() {
  countR++;
}


/* =========================================================
   Initialization routine
   ========================================================= */
void setup() {
  Serial.begin(115200);      // Serial communication for debugging and logging
  Wire.begin();              // Initialize I2C bus

  // Configure encoder pins as pull-up inputs
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);

  // Attach interrupt handlers to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), ISR_right, RISING);

  // Initialize the MPU6050 sensor
  mpu.initialize();

  /* ---------- Gyroscope Z-axis bias calibration ---------- */
  long sum = 0;              // Accumulator for raw gyro readings
  for (int i = 0; i < 500; i++) {
    int16_t gz;              // Raw Z-axis gyroscope reading
    mpu.getRotation(nullptr, nullptr, &gz);
    sum += gz;
    delay(2);
  }

  // Compute average bias
  gyroZ_offset = sum / 500.0;

  // Initialize time reference
  lastTime = millis();
}


/* =========================================================
   Main control loop
   ========================================================= */
void loop() {

  /* ---------- Time update ---------- */

  // Current time in milliseconds
  unsigned long now = millis();

  // Time step in seconds
  float dt = (now - lastTime) / 1000.0;

  // Update timestamp
  lastTime = now;

  // Safety check against invalid time step
  if (dt <= 0) return;


  /* ---------- Encoder delta computation ---------- */

  // Previous encoder counts (static to persist across loops)
  static long lastL = 0;
  static long lastR = 0;

  // Left wheel pulse increment during this time step
  long dL = countL - lastL;

  // Right wheel pulse increment during this time step
  long dR = countR - lastR;

  // Update stored counts
  lastL = countL;
  lastR = countR;


  /* ---------- Wheel speed and linear velocity ---------- */

  // Wheel circumference in meters
  float wheelCirc = PI * WHEEL_DIAMETER;

  // Left wheel linear velocity (m/s)
  float vL = (dL / (float)CPR) * wheelCirc / dt;

  // Right wheel linear velocity (m/s)
  float vR = (dR / (float)CPR) * wheelCirc / dt;

  // Robot forward linear velocity (m/s)
  float v = (vL + vR) * 0.5;


  /* ---------- Yaw estimation using gyroscope ---------- */

  // Raw Z-axis gyroscope reading
  int16_t gz;

  // Read gyroscope data from MPU6050
  mpu.getRotation(nullptr, nullptr, &gz);

  // Convert raw value to angular velocity (deg/s)
  float gyroZ = (gz - gyroZ_offset) / 131.0;

  // Integrate angular velocity to obtain yaw angle (rad)
  yaw += gyroZ * dt * DEG_TO_RAD;


  /* ---------- Position integration ---------- */

  // Update x position in world frame
  x += v * cos(yaw) * dt;

  // Update y position in world frame
  y += v * sin(yaw) * dt;


  /* ---------- Telemetry output ---------- */

  Serial.print("x: ");
  Serial.print(x, 3);              // x position (meters)

  Serial.print(" y: ");
  Serial.print(y, 3);              // y position (meters)

  Serial.print(" yaw(deg): ");
  Serial.println(yaw * RAD_TO_DEG); // yaw angle (degrees)

  delay(10);   // Loop rate control (~100 Hz)
}
