#include <Wire.h>
#include <MPU6050.h>

/* =========================================================
   ===================== PIN DEFINITIONS ====================
   ========================================================= */
const int STEP_L = 2;
const int DIR_L  = 3;
const int STEP_R = 4;
const int DIR_R  = 5;

/* =========================================================
   ===================== IMU ================================
   ========================================================= */
MPU6050 mpu;

float yaw = 0.0;             // Current yaw angle (rad)
float gyroZ_bias = 0.0;      // Gyro Z-axis bias (raw units)

/* =========================================================
   ===================== ROBOT PARAMETERS ===================
   ========================================================= */
const float wheelRadius = 0.0381;   // Wheel radius (m) - 3 inch diameter
const float wheelBase   = 0.18;     // Distance between left and right wheels (m)

const int stepsPerRev = 200;         // Full steps per revolution
const int microStep  = 16;           // Microstepping factor

const float stepsPerRad =
  (stepsPerRev * microStep) / (2.0 * PI);

/* =========================================================
   ===================== POSITION ESTIMATION =================
   ========================================================= */
float posX = 0.0;   // Robot X position in world frame (m)
float posY = 0.0;   // Robot Y position in world frame (m)

/* =========================================================
   ===================== YAW PID CONTROLLER =================
   ========================================================= */
float kp_yaw = 3.0;
float ki_yaw = 0.0;
float kd_yaw = 0.1;

float yawErrPrev = 0.0;
float yawErrInt  = 0.0;

/* =========================================================
   ===================== SEGMENT DEFINITION =================
   ========================================================= */
struct Segment {
  float v_max;       // Maximum forward speed (m/s)
  float targetX;     // Target X position (m)
  float targetY;     // Target Y position (m)
  float targetYaw;   // Target yaw at the end (rad)
};

/* Example path */
Segment path[] = {
  {0.6, 1.0, 0.0, 0.0},          // Move to (1.0, 0.0)
  {0.6, 1.0, 1.0, PI / 2},       // Move to (1.0, 1.0)
  {0.6, 0.0, 1.0, PI}            // Move to (0.0, 1.0)
};

const int NUM_SEGMENTS = sizeof(path) / sizeof(path[0]);
int currentSegment = 0;

/* =========================================================
   ===================== STEPPER TIMING =====================
   ========================================================= */
unsigned long lastStepTimeL = 0;
unsigned long lastStepTimeR = 0;

/* =========================================================
   ===================== TIME MANAGEMENT ====================
   ========================================================= */
unsigned long lastLoopTime = 0;

/* =========================================================
   ===================== SETUP ==============================
   ========================================================= */
void setup() {
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  Serial.begin(115200);

  Wire.begin();
  mpu.initialize();
  calibrateGyro();

  lastLoopTime = micros();
}

/* =========================================================
   ===================== MAIN LOOP ==========================
   ========================================================= */
void loop() {

  /* ---------- Time Step ---------- */
  unsigned long now = micros();
  float dt = (now - lastLoopTime) * 1e-6;
  lastLoopTime = now;

  if (dt <= 0 || dt > 0.05) return;

  /* ---------- Update Yaw ---------- */
  updateYaw(dt);

  /* ---------- Check Path End ---------- */
  if (currentSegment >= NUM_SEGMENTS) {
    stopMotors();
    return;
  }

  Segment &seg = path[currentSegment];

  /* ---------- Compute Position Error ---------- */
  float dx = seg.targetX - posX;
  float dy = seg.targetY - posY;
  float distanceError = sqrt(dx * dx + dy * dy);

  /* ---------- Target Yaw ---------- */
  float yaw_target;
  if (distanceError > 0.05) {
    yaw_target = atan2(dy, dx);
  } else {
    yaw_target = seg.targetYaw;
  }

  float yaw_error = wrapAngle(yaw_target - yaw);

  /* ---------- Arrival Speed Control ---------- */
  float k_v = 1.2;            // Speed decay gain
  float v_min = 0.05;         // Minimum speed (m/s)

  float v_cmd = k_v * distanceError;
  if (v_cmd > seg.v_max) v_cmd = seg.v_max;
  if (v_cmd < v_min) v_cmd = v_min;

  if (distanceError < 0.02) {
    v_cmd = 0.0;
  }

  /* ---------- Segment Completion ---------- */
  bool positionReached = distanceError < 0.02;
  bool yawReached = abs(wrapAngle(seg.targetYaw - yaw)) < 2.0 * DEG_TO_RAD;

  if (positionReached && yawReached) {
    currentSegment++;
    yawErrInt = 0.0;
    yawErrPrev = 0.0;
    return;
  }

  /* ---------- Yaw PID ---------- */
  yawErrInt += yaw_error * dt;
  float yawErrDer = (yaw_error - yawErrPrev) / dt;
  yawErrPrev = yaw_error;

  float omega_cmd =
    kp_yaw * yaw_error +
    ki_yaw * yawErrInt +
    kd_yaw * yawErrDer;

  /* ---------- Differential Drive ---------- */
  float v_l = v_cmd - omega_cmd * wheelBase / 2.0;
  float v_r = v_cmd + omega_cmd * wheelBase / 2.0;

  /* ---------- Position Update ---------- */
  float v_center = (v_l + v_r) / 2.0;
  posX += v_center * cos(yaw) * dt;
  posY += v_center * sin(yaw) * dt;

  /* ---------- Drive Steppers ---------- */
  driveStepper(v_l, v_r);

  /* ---------- Debug Output ---------- */
  Serial.print("X: "); Serial.print(posX, 3);
  Serial.print(" Y: "); Serial.print(posY, 3);
  Serial.print(" Yaw(deg): "); Serial.println(yaw * 180.0 / PI);
}

/* =========================================================
   ===================== FUNCTIONS ==========================
   ========================================================= */

void updateYaw(float dt) {
  int16_t gz = mpu.getRotationZ();
  float gyroZ = (gz - gyroZ_bias) * 0.0010653; // rad/s
  yaw += gyroZ * dt;
}

void driveStepper(float v_l, float v_r) {
  setStepper(STEP_L, DIR_L, v_l, lastStepTimeL);
  setStepper(STEP_R, DIR_R, v_r, lastStepTimeR);
}

void setStepper(int stepPin, int dirPin,
                float v, unsigned long &lastTime) {

  bool dir = (v >= 0);
  digitalWrite(dirPin, dir);

  float omega = abs(v) / wheelRadius;
  float stepRate = omega * stepsPerRad;

  if (stepRate < 1) return;

  unsigned long interval = 1000000.0 / stepRate;
  unsigned long now = micros();

  if (now - lastTime >= interval) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    lastTime = now;
  }
}

void stopMotors() {
  // No step pulses = motor stopped
}

void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    sum += mpu.getRotationZ();
    delay(2);
  }
  gyroZ_bias = sum / 1000.0;
}

float wrapAngle(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}
